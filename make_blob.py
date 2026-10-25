#!/usr/bin/env python3
# make_blob.py — 系数 JSON -> BAP2 容器 (int16 量化 + 帧间差分 + 分块 xz)
#
# 两刀:
#   1) float32 -> int16 定点 (Q8: ×256), 量化误差 ~0.02px, 远低于 24 谐波截断的 0.54px
#   2) 相邻帧同序号轮廓做差分 (residual), 首帧/换形/溢出自动回退绝对编码
#
# 容器布局 (小端):
#   'BAP2' u32 W u32 H u32 FPS u32 TOTAL u32 NCHUNK u32 CHUNK_FRAMES
#   NCHUNK x { u32 comp_off, comp_len, raw_len, frame_first }   # comp_off 自 blob 头起算
#   之后为 NCHUNK 条互相独立的 xz 流顺序拼接 (每块首帧为 KEY, 块间零依赖 -> 可并行解码)
#
# 块内帧记录:
#   u16 (cnt | 0x8000=KEY)
#   每轮廓: u8 (k0+64 | 0x80=ABS)  u8 (k1+64)  i16 x 2*(k1-k0+1)   # 残差或绝对
#
# 用法:  python3 make_blob.py badapple.json badapple_blob.bin
# 嵌入:  ld -r -b binary -o blob.o badapple_blob.bin
#        gcc -O2 -march=native badapple.c xz_embedded/*.c blob.o -o badapple -lm -lpthread
import json, lzma, struct, sys
import numpy as np

Q = 256                 # 定点小数位数
CHUNK_FRAMES = 128      # 每块帧数: 块 = 并行解码/按需解码的最小单位
DICT_SIZE = 2 << 20     # xz 字典 2MiB, 覆盖最肥的块绰绰有余
K0, K1 = -24, 24
NC = 2 * (K1 - K0 + 1)  # 每轮廓系数对数 x2 = 98


def quantize(frame):
    """帧内全部轮廓 -> int16 域 int32 数组列表 (绝对值)"""
    out = []
    for c in frame['f']:
        a = np.round(np.array(c['c'], dtype=np.float64) * Q).astype(np.int32).reshape(-1)
        assert a.max() <= 32767 and a.min() >= -32767, 'coefficient overflow'
        k0, k1 = c['k']
        assert (k0, k1) == (K0, K1) and c['n'] == 256, 'unexpected contour shape'
        out.append(a)
    return out


def encode_frame(cur, ref, ref_k, force_key):
    """一帧 -> bytes; 同步更新参考系 (与 C 解码器严格镜像; KEY 帧重建参考系)"""
    assert len(cur) <= 512, 'contour count exceeds decoder MAXCT'
    if force_key:
        ref.clear()
        ref_k.clear()
    key = force_key or not ref
    parts = [struct.pack('<H', len(cur) | (0x8000 if key else 0))]
    for j, a in enumerate(cur):
        use_ref = (not key) and j < len(ref) \
            and ref_k[j] is not None and ref_k[j] == (K0, K1)
        r = a - ref[j] if use_ref else a
        if use_ref and (np.abs(r).max() > 32767):
            r, use_ref = a, False
        b0 = (K0 + 64) | (0 if use_ref else 0x80)
        parts.append(struct.pack('<BB', b0, K1 + 64))
        parts.append(r.astype('<i2').tobytes())
    # 参考系维护: 覆盖已有槽位, 追加新槽位
    while len(ref) < len(cur):
        ref.append(None)
        ref_k.append(None)
    for j, a in enumerate(cur):
        ref[j] = a.copy()
        ref_k[j] = (K0, K1)
    return b''.join(parts)


def undiff(chunk, total_ref, total_ref_k):
    """解码器镜像: 块内 bytes -> 绝对系数帧列表 (自检用)"""
    ref, ref_k = [], []
    frames, pos = [], 0
    while pos < len(chunk):
        (hdr,) = struct.unpack_from('<H', chunk, pos)
        pos += 2
        key = bool(hdr & 0x8000)
        if key:
            ref, ref_k = [], []
        cur = []
        for _ in range(hdr & 0x7FFF):
            b0, b1 = struct.unpack_from('<BB', chunk, pos)
            pos += 2
            k0, k1 = (b0 & 0x7F) - 64, b1 - 64
            n = 2 * (k1 - k0 + 1)
            r = np.frombuffer(chunk, '<i2', n, pos).astype(np.int32)
            pos += 2 * n
            j = len(cur)
            if (b0 & 0x80) or j >= len(ref):
                a = r
            else:
                a = ref[j] + r
            cur.append(a)
            if j < len(ref):
                ref[j] = a.copy()
                ref_k[j] = (k0, k1)
            else:
                ref.append(a.copy())
                ref_k.append((k0, k1))
        frames.append(cur)
    return frames


def make_blob(in_json, out_bin):
    d = json.load(open(in_json))
    frames = d['frames']
    total = len(frames)
    quants = [quantize(f) for f in frames]

    chunks = []
    ref, ref_k = [], []          # 参考系跨块持续, 与解码端一致 (块首 KEY 重建)
    for lo in range(0, total, CHUNK_FRAMES):
        body = bytearray()
        for i in range(lo, min(lo + CHUNK_FRAMES, total)):
            body += encode_frame(quants[i], ref, ref_k, force_key=(i == lo))
        chunks.append(bytes(body))

    # 自检: 解码镜像必须逐值还原量化绝对值
    for ci, body in enumerate(chunks):
        got = undiff(body, ref, ref_k)
        want = quants[ci * CHUNK_FRAMES:(ci + 1) * CHUNK_FRAMES]
        assert len(got) == len(want), f'chunk {ci}: frame count mismatch'
        for a, b in zip(got, want):
            assert len(a) == len(b), 'contour count mismatch'
            for x, y in zip(a, b):
                assert np.array_equal(x, y), 'undiff mismatch'

    table = bytearray()
    streams = []
    off = 4 + 6 * 4 + 16 * ((total + CHUNK_FRAMES - 1) // CHUNK_FRAMES)
    for ci, body in enumerate(chunks):
        comp = lzma.compress(body, format=lzma.FORMAT_XZ, check=lzma.CHECK_CRC32,
                             filters=[{'id': lzma.FILTER_LZMA2,
                                       'preset': 9 | lzma.PRESET_EXTREME,
                                       'dict_size': DICT_SIZE}])
        table += struct.pack('<IIII', off, len(comp), len(body), ci * CHUNK_FRAMES)
        streams.append(comp)
        off += len(comp)
        print(f'chunk {ci}: {len(body)} -> {len(comp)} bytes')

    blob = b'BAP2' + struct.pack('<6I', d['w'], d['h'], d['fps'], total,
                                 len(chunks), CHUNK_FRAMES) + bytes(table) + b''.join(streams)
    open(out_bin, 'wb').write(blob)
    raw = sum(len(c) for c in chunks)
    print(f'{out_bin}: {len(blob)} bytes (raw {raw}, x{raw/len(blob):.1f}), '
          f'{total} frames, {len(chunks)} chunks, Q={Q}')


if __name__ == '__main__':
    make_blob(sys.argv[1] if len(sys.argv) > 1 else 'badapple.json',
              sys.argv[2] if len(sys.argv) > 2 else 'badapple_blob.bin')
