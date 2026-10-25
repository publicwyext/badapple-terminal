#!/usr/bin/env python3
# encode_planb.py — 视频 -> 系数 JSON (无 cv2 依赖)
# 流程: ffmpeg 抽原始灰度帧 -> Otsu 二值 -> 像素裂缝追踪闭合轮廓 -> 弧长重采样 -> FFT 截断谐波
# 输出格式: {"w","h","fps","k","frames":[{"f":[{"n":256,"k":[-K,K],"c":[[re,im]*49]}]}]}
# 用法:
#   python3 encode_planb.py <video> <out.json>                # 编码
#   python3 encode_planb.py --sanitize <src.json> <dst.json> [L,R,T]
#       从系数 JSON 剜除舞台相框环 (852 压制版专用; 真原版 480x360 无框, 不需要)
import json, subprocess, sys
import numpy as np

N_SAMPLES = 256      # 每轮廓重采样点数
K_HARMONICS = 24     # 保留谐波数: k ∈ [-24, 24]

def otsu(gray):
    hist = np.bincount(gray.ravel(), minlength=256).astype(float)
    total = hist.sum(); lv = np.arange(256)
    w0 = np.cumsum(hist); w1 = total - w0
    m0 = np.cumsum(hist * lv)
    mt = m0[-1]
    valid = (w0 > 0) & (w1 > 0)
    var = np.where(valid, (m0*total - mt*w0)**2 / np.maximum(w0*w1, 1e-12)**2, 0)
    return int(np.argmax(var))

def trace_loops(ink):
    """沿像素裂缝追踪闭合边界, 返回折线列表(浮点格点)"""
    from collections import defaultdict
    out = defaultdict(list)
    ys, xs = np.nonzero(ink)
    W, H = ink.shape[1], ink.shape[0]
    def ink_(x, y): return 0 <= x < W and 0 <= y < H and ink[y, x]
    for y, x in zip(ys.tolist(), xs.tolist()):
        if not ink_(x, y-1): out[(x, y)].append((1, 0))
        if not ink_(x, y+1): out[(x+1, y+1)].append((-1, 0))
        if not ink_(x-1, y): out[(x, y+1)].append((0, -1))
        if not ink_(x+1, y): out[(x+1, y)].append((0, 1))
    loops = []
    for s in list(out.keys()):
        while out[s]:
            d0 = out[s].pop()
            pts = [s]
            cv = (s[0]+d0[0], s[1]+d0[1]); cd = d0
            while cv != s:
                cand = out[cv]
                if not cand: break
                rx, ry = cd[1], -cd[0]           # 优先右转, 保证鞍点处环路不串
                pick = next((i for i, d in enumerate(cand) if d == (rx, ry)), None)
                if pick is None:
                    pick = next((i for i, d in enumerate(cand) if d == cd), 0)
                dd = cand.pop(pick)
                cv = (cv[0]+dd[0], cv[1]+dd[1]); cd = dd
                if cv != s:
                    pts.append(cv)
            else:
                # while 自然结束(cv==s)才是真闭合环; 断环丢掉, 否则谐波会补一条"弦"
                if len(pts) >= 4:
                    loops.append(np.array(pts, float))
    return loops

def poly_to_coef(pts, n=N_SAMPLES, K=K_HARMONICS):
    """闭合折线 -> 沿弧长均匀重采样 n 点 -> FFT 截断 k∈[-K,K] -> 系数对"""
    seg = np.vstack([pts, pts[:1]]).astype(float)    # 闭合
    d = np.hypot(*(seg[1:] - seg[:-1]).T)
    s = np.concatenate([[0.0], np.cumsum(d)])
    total = s[-1]
    if total <= 0:
        total = 1.0
    u = np.arange(n) * (total / n)
    z = np.interp(u, s, seg[:, 0]) + 1j * np.interp(u, s, seg[:, 1])
    C = np.fft.fft(z) / n                             # c_k = (1/n) Σ z_s e^{-2πi k s/n}
    idx = np.arange(-K, K + 1) % n
    cf = C[idx]
    return {"n": n, "k": [-K, K], "c": [[float(v.real), float(v.imag)] for v in cf]}

def loop_pts(c):
    """系数 dict -> 256 点闭合折线 (反演, 供手术/自检)"""
    KS = np.arange(-K_HARMONICS, K_HARMONICS + 1)
    A = np.zeros(N_SAMPLES, complex)
    cc = np.array(c['c']).ravel()
    A[KS % N_SAMPLES] = cc[0::2] + 1j * cc[1::2]
    z = np.fft.ifft(A) * N_SAMPLES
    return np.stack([z.real, z.imag], 1)

def sanitize(src, dst, bar=(13, 83, 3), SS=4):
    """一次性手术: 从系数 JSON 剜掉舞台相框环
    相框 (左 L 列 / 右 R 列起 / 上 T 行) 随黑白反转换边, 总有一头落进墨里:
      - 相框单独成环 -> 两侧神秘竖条
      - 人物贴框时与相框焊成一条环 -> 贴边帧边框复活
    手法: 受影响帧反演墨图 (SS 超采样栅格化) -> 抹掉环区 -> 重追环 -> 重编码系数;
    未受影响帧保持原系数不动 (零损伤)"""
    from PIL import Image, ImageDraw
    L_BAR, R_BAR, T_BAR = bar
    d = json.load(open(src))
    touched = loops_before = loops_after = 0
    out = []
    for fi, f in enumerate(d['frames']):
        polys = [loop_pts(c) for c in f['f']]
        hit = any(p[:, 0].min() < L_BAR + 0.3 or p[:, 0].max() > R_BAR - 0.3
                  or p[:, 1].min() < T_BAR + 0.3 for p in polys)
        if not hit:
            out.append(f); loops_before += len(polys); loops_after += len(polys)
            continue
        touched += 1
        img = Image.new('L', (d['w'] * SS, d['h'] * SS), 0)
        dr = ImageDraw.Draw(img)
        for p in polys:
            dr.polygon([((x + 0.5) * SS, (y + 0.5) * SS) for x, y in p], fill=255)
        ink = np.array(img.resize((d['w'], d['h']), Image.BOX), float) >= 127.5
        ink[:, :L_BAR] = False; ink[:, R_BAR:] = False; ink[:T_BAR, :] = False
        loops = trace_loops(ink)
        loops_before += len(polys); loops_after += len(loops)
        nf = {"f": [poly_to_coef(p) for p in loops]}
        if f.get("inv"): nf["inv"] = True
        out.append(nf)
        if fi % 1000 == 0: print('frame', fi, flush=True)
    d['frames'] = out
    json.dump(d, open(dst, 'w'))
    print(f'touched {touched} frames, loops {loops_before} -> {loops_after}, saved {dst}')

def encode(in_path, out_json, W=96, H=48):
    proc = subprocess.Popen(
        ['ffmpeg', '-v', 'error', '-i', in_path, '-vf', f'scale={W}:{H}',
         '-pix_fmt', 'gray', '-f', 'rawvideo', '-'],
        stdout=subprocess.PIPE)
    frames = []
    while True:
        buf = proc.stdout.read(W*H)
        if not buf or len(buf) < W*H: break
        g = np.frombuffer(buf, np.uint8).reshape(H, W)
        thr = otsu(g)
        ink = g < thr                     # 黑为墨
        fr = {"f": []}
        if ink.mean() > 0.5:              # 黑主导 -> 黑底白影
            ink = ~ink
            fr["inv"] = True
        fr["f"] = [poly_to_coef(p) for p in trace_loops(ink)]
        frames.append(fr)
        if len(frames) % 500 == 0:
            print('frame', len(frames), flush=True)
    proc.wait()
    json.dump({'w': W, 'h': H, 'fps': 30, 'k': K_HARMONICS,
               'frames': frames}, open(out_json, 'w'))
    print('saved', out_json, len(frames), 'frames')

if __name__ == '__main__':
    args = sys.argv[1:]
    if args and args[0] == '--sanitize':
        bar = tuple(int(v) for v in args[3].split(',')) if len(args) > 3 else (13, 83, 3)
        sanitize(args[1], args[2], bar)
    else:
        encode(args[0] if args else 'badapple.mp4',
               args[1] if len(args) > 1 else 'badapple.json')
