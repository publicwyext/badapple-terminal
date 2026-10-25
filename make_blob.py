#!/usr/bin/env python3
# make_blob.py — 系数 JSON -> BAP3 容器 (稀疏化 + 质心匹配 + 相位对齐 + 槽位差分 + 分块 xz)
#
# 五刀:
#   1) float32 -> int16 定点 (Q8: x256); |c| <= ZERO(32=0.125px) 的高次谐波清零 —
#      远低于既有的 0.54px FFT 截断误差, 却让稀疏度与残差同时爆炸 (blob -57%)
#   2) 轮廓按质心 (c[0],c[1]) 最近邻匹配参考槽 (12px 闸门, LRU 回收), 不再依赖序号
#   3) 匹配后做相位对齐: 256 点 FFT 互相关找起点相位差 + 方向翻转, 残差按对齐后差分
#   4) 对齐残差 max <= RGATE(2048=8px) 走差分, 否则整体绝对值; 两态都占槽更新参考
#   5) xz 数据校验 CHECK_NONE (块头/索引 CRC32 仍在), LZMA2 9e / 2MiB dict
#
# 容器布局 (小端):
#   'BAP3' u32 W u32 H u32 FPS u32 TOTAL u32 NCHUNK u32 CHUNK_FRAMES
#   NCHUNK x { u32 comp_off, comp_len, raw_len, frame_first }
#   之后 NCHUNK 条独立 xz 流顺序拼接 (每块首帧 KEY 清空参考系, 块间零依赖)
#
# 块内帧记录:
#   u16 (cnt | 0x8000=KEY)
#   每轮廓: u8 (k0+64 | 0x80=ABS)  u8 (k1+64)  u8 slot (0..254; 0xFF 保留)
#   i16 x 2*(k1-k0+1)   # 非 ABS 且槽位有效 -> 残差; 否则绝对值 (两种都写回槽位)
#
# 用法:  python3 make_blob.py badapple.json badapple_blob.bin
# 嵌入:  ld -r -b binary -o blob.o badapple_blob.bin
import json, lzma, struct, sys
import numpy as np

Q = 256                 # 定点小数位数
CHUNK_FRAMES = 128
DICT_SIZE = 2 << 20
K0, K1 = -24, 24
NCOEF = 2 * (K1 - K0 + 1)   # 98
NSLOT = 255             # u8 槽位; 0xFF 保留
GATE2 = 12.0 ** 2       # 质心最近邻闸门 (源像素距离平方)
RGATE = 2048            # 对齐残差 max 闸门 (量化单位; 2048 = 8px)
ZERO = 32               # |量化系数| <= ZERO 清零 (32 = 0.125px, << 0.54px 截断误差)
NP_ = 256

_KS = np.arange(K0, K1 + 1)
_IDX = _KS % NP_
_ALL = np.arange(NP_)


def _padded(c98):
    A = np.zeros(NP_, complex)
    A[_IDX] = c98[0::2] + 1j * c98[1::2]
    return A


def align_rot(ref98, cur98):
    """起点相位 + 方向对齐: 返回对齐后的 cur98 (与 ref98 同相位基准)"""
    refA = _padded(ref98)
    cr, ci = cur98[0::2], cur98[1::2]
    best = None
    for flip in (False, True):
        if flip:   # 反向遍历: c'_k = conj(c_k) * e^{-2πik/N}
            Bp = np.zeros(NP_, complex)
            Bp[_IDX] = (cr - 1j * ci) * np.exp(-2j * np.pi * _KS / NP_)
        else:
            Bp = _padded(cur98)
        corr = np.fft.ifft(np.conj(refA) * Bp).real
        dl = int(np.argmax(corr))
        rot = (Bp * np.exp(-2j * np.pi * _ALL * dl / NP_))[_IDX]
        resid = np.abs(rot - refA[_IDX]).max()
        if best is None or resid < best:
            best = resid
            brot = rot
    o = np.empty(98)
    o[0::2] = brot.real
    o[1::2] = brot.imag
    return o


def encode_frame(quants, slots, key, fno):
    """一帧 -> (bytes, 期望绝对值列表); 就地维护 255 槽参考系 (与 C 解码器严格镜像)
    quants: 浮点系数 (float98); slots[s] = None 或 {'q': int32-98, 'cen': (cx, cy)}"""
    assert len(quants) <= 511, 'contour count exceeds frame header'
    if key:
        for s in range(NSLOT):
            slots[s] = None
    parts = [struct.pack('<H', len(quants) | (0x8000 if key else 0))]
    expect = []
    for a in quants:
        cx, cy = a[0], a[1]
        use, best = -1, GATE2
        for s in range(NSLOT):
            sl = slots[s]
            if sl is None:
                continue
            dx = cx - sl['cen'][0]
            dy = cy - sl['cen'][1]
            d2 = dx * dx + dy * dy
            if d2 <= best:
                best = d2
                use = s
        aligned = None
        if use >= 0:
            rot = align_rot(slots[use]['q'] / Q, a)
            aq = np.round(rot * Q).astype(np.int32)
            aq[np.abs(aq) <= ZERO] = 0
            r = aq - slots[use]['q']
            if np.abs(r).max() <= RGATE:
                aligned = aq
                is_abs = False
        if aligned is None:
            aq = np.round(a * Q).astype(np.int32)
            aq[np.abs(aq) <= ZERO] = 0
            claim = next((s for s in range(NSLOT) if slots[s] is None), -1)
            if claim < 0:   # 槽满: 回收最久未更新者 (仅编码端策略, 解码端无感)
                claim = min(range(NSLOT), key=lambda s: slots[s]['seen'])
            use = claim
            aligned = aq
            is_abs = True
        abs_flag = 0x80 if is_abs else 0
        parts.append(bytes(((K0 + 64) | abs_flag, K1 + 64, use)))
        val = aligned if is_abs else aligned - slots[use]['q']
        parts.append(val.astype('<i2').tobytes())
        expect.append(aligned)
        slots[use] = {'q': aligned.copy(), 'cen': (aligned[0] / Q, aligned[1] / Q),
                      'seen': fno}
    return b''.join(parts), expect


def undiff(chunk):
    """解码器镜像: 块内 bytes -> 绝对系数帧列表 (自检用)"""
    ref = [None] * 256
    frames, pos = [], 0
    while pos < len(chunk):
        (hdr,) = struct.unpack_from('<H', chunk, pos)
        pos += 2
        if hdr & 0x8000:
            ref = [None] * 256
        cur = []
        for _ in range(hdr & 0x7FFF):
            b0, b1, slot = chunk[pos], chunk[pos + 1], chunk[pos + 2]
            pos += 3
            k0, k1 = (b0 & 0x7F) - 64, b1 - 64
            n = 2 * (k1 - k0 + 1)
            r = np.frombuffer(chunk, '<i2', n, pos).astype(np.int32)
            pos += 2 * n
            if slot != 0xFF and not b0 & 0x80 and ref[slot] is not None:
                a = ref[slot] + r
            else:
                a = r
            cur.append(a)
            if slot != 0xFF:
                ref[slot] = a.copy()
        frames.append(cur)
    return frames


def make_blob(in_json, out_bin):
    d = json.load(open(in_json))
    frames = d['frames']
    total = len(frames)
    quants = []
    for f in frames:
        fq = []
        for c in f['f']:
            a = np.array(c['c'], dtype=np.float64).reshape(-1)
            k0, k1 = c['k']
            assert (k0, k1) == (K0, K1) and c['n'] == 256, 'unexpected contour shape'
            fq.append(a)
        quants.append(fq)

    chunks = []
    slots = [None] * NSLOT
    expects = []
    for lo in range(0, total, CHUNK_FRAMES):
        body = bytearray()
        for i in range(lo, min(lo + CHUNK_FRAMES, total)):
            body_, exp = encode_frame(quants[i], slots, key=(i == lo), fno=i)
            body += body_
            expects.append(exp)
        chunks.append(bytes(body))

    # 自检: 解码镜像逐值还原 == 编码端的期望绝对值 (对齐+零化后)
    for ci, body in enumerate(chunks):
        got = undiff(body)
        chunk_exp = expects[ci * CHUNK_FRAMES:(ci + 1) * CHUNK_FRAMES]
        assert len(got) == len(chunk_exp), f'chunk {ci}: frame count mismatch'
        for a, b in zip(got, chunk_exp):
            assert len(a) == len(b), 'contour count mismatch'
            assert np.array_equal(a, b), 'undiff mismatch'

    table = bytearray()
    streams = []
    off = 4 + 6 * 4 + 16 * ((total + CHUNK_FRAMES - 1) // CHUNK_FRAMES)
    for ci, body in enumerate(chunks):
        comp = lzma.compress(body, format=lzma.FORMAT_XZ, check=lzma.CHECK_NONE,
                             filters=[{'id': lzma.FILTER_LZMA2,
                                       'preset': 9 | lzma.PRESET_EXTREME,
                                       'dict_size': DICT_SIZE}])
        table += struct.pack('<IIII', off, len(comp), len(body), ci * CHUNK_FRAMES)
        streams.append(comp)
        off += len(comp)
        print(f'chunk {ci}: {len(body)} -> {len(comp)} bytes')

    blob = b'BAP3' + struct.pack('<6I', d['w'], d['h'], d['fps'], total,
                                 len(chunks), CHUNK_FRAMES) + bytes(table) + b''.join(streams)
    open(out_bin, 'wb').write(blob)
    raw = sum(len(c) for c in chunks)
    print(f'{out_bin}: {len(blob)} bytes (raw {raw}, x{raw/len(blob):.1f}), '
          f'{total} frames, {len(chunks)} chunks, Q={Q} ZERO={ZERO} check=NONE')


if __name__ == '__main__':
    make_blob(sys.argv[1] if len(sys.argv) > 1 else 'badapple.json',
              sys.argv[2] if len(sys.argv) > 2 else 'badapple_blob.bin')
