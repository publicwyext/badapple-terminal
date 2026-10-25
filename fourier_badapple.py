#!/usr/bin/env python3
# fourier_badapple.py — Bad Apple 傅里叶描述子管线
# 用法:
#   python3 fourier_badapple.py encode badapple.mp4 out.json   # 需 opencv-python-headless
#   python3 fourier_badapple.py play out.json                  # 控制台播放, 仅需 numpy
#   python3 fourier_badapple.py demo                           # 无视频单帧自检
import json, math, sys, time
import numpy as np

GRID_W, GRID_H = 96, 48   # 播放网格
HARMONICS = 24            # 每条轮廓保留 ±K 次谐波
FPS = 30.0

# ---------- 公共: even-odd 扫描线填充 ----------
def fill_evenodd(polys, w, h):
    mask = np.zeros((h, w), bool)
    bx1, by1, bx2, by2 = [], [], [], []
    for pts in polys:
        if len(pts) < 3:
            continue
        x1, y1 = pts[:, 0], pts[:, 1]
        bx1.append(x1); by1.append(y1)          # 每条轮廓独立闭合,
        bx2.append(np.roll(x1, -1))             # 严禁跨轮廓成弦
        by2.append(np.roll(y1, -1))
    if not bx1:
        return mask
    x1 = np.concatenate(bx1); y1 = np.concatenate(by1)
    x2 = np.concatenate(bx2); y2 = np.concatenate(by2)
    keep = y1 != y2                       # 水平边不贡献交点
    x1, y1, x2, y2 = x1[keep], y1[keep], x2[keep], y2[keep]
    dx = x2 - x1
    dy = y2 - y1
    for y in range(h):
        cross = ((y1 <= y) & (y < y2)) | ((y2 <= y) & (y < y1))
        if not cross.any():
            continue
        xs = x1[cross] + (y - y1[cross]) * dx[cross] / dy[cross]
        xs.sort()
        lo = np.maximum(xs[::2], 0).astype(np.int64)
        hi = np.minimum(xs[1::2], w - 1).astype(np.int64)
        for a, b in zip(lo, hi):
            if a <= b:
                mask[y, a:b + 1] = True
    return mask

# ---------- 编码: 折线 -> ±K 谐波系数 ----------
def poly_to_coef(pts, k=HARMONICS, N=256):
    z = pts[:, 0] + 1j * pts[:, 1]
    d = np.abs(np.diff(np.r_[z, z[:1]]))
    s = np.r_[0, np.cumsum(d)]
    si = np.linspace(0, s[-1], N, endpoint=False)          # 弧长重采样
    zr = (np.interp(si, s, np.r_[z.real, z.real[:1]])
          + 1j * np.interp(si, s, np.r_[z.imag, z.imag[:1]]))
    F = np.fft.fft(zr) / N
    ks = np.arange(-k, k + 1)
    co = np.array([[F[int(kk) % N].real, F[int(kk) % N].imag] for kk in ks])
    return {"n": N, "k": [-k, k], "c": np.round(co, 4).tolist()}

_PH_CACHE = {}
def decode_contour(coef, samples=900):
    N = coef["n"]; k0, k1 = coef["k"]; C = np.array(coef["c"])
    key = (samples, k0, k1)
    ks = np.arange(k0, k1 + 1)
    ph = _PH_CACHE.get(key)
    if ph is None:
        t = np.arange(samples) / samples
        ph = np.exp(2j * np.pi * np.outer(t, ks))
        _PH_CACHE[key] = ph
    z = ph @ (C[:, 0] + 1j * C[:, 1])
    return np.c_[z.real, z.imag]

# ---------- 视频 -> JSON ----------
def encode_video(path, out_json):
    import cv2
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or FPS
    frames = []
    while True:
        ok, img = cap.read()
        if not ok:
            break
        g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        g = cv2.resize(g, (GRID_W, GRID_H), interpolation=cv2.INTER_AREA)
        _, bw = cv2.threshold(g, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        ink = bw == 0                              # 黑为墨
        fr = {"f": []}
        if ink.mean() > 0.5:                       # 黑主导 -> 黑底白影
            ink = ~ink
            fr["inv"] = True
        cs, _ = cv2.findContours((ink * 255).astype(np.uint8),
                                 cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
        fr["f"] = [poly_to_coef(c[:, 0, :].astype(float))
                   for c in cs if abs(cv2.contourArea(c)) >= 3]
        frames.append(fr)
        if len(frames) % 500 == 0:
            print("frame", len(frames), flush=True)
    json.dump({"w": GRID_W, "h": GRID_H, "fps": fps,
               "k": HARMONICS, "frames": frames}, open(out_json, "w"))
    print("saved", out_json, len(frames), "frames")

# ---------- 控制台播放 ----------
def _frame_polys(fr):
    """兼容旧版(list)与新版(dict, 含 inv 标记)两种帧结构"""
    if isinstance(fr, dict):
        return fr.get("f", []), bool(fr.get("inv", False))
    return fr, False

def play(path, fps=None, scale=None):
    import queue, shutil, threading
    d = json.load(open(path))
    w0, h0 = d["w"], d["h"]
    cols, rows = shutil.get_terminal_size((220, 50))
    if scale is None:                   # 屏幕自适应: 逐级放大到塞满终端
        scale = 1
        if cols >= 2 * w0:              # 放得下双字符就优先双字符(比例正)
            while cols >= 2 * w0 * (scale + 1) and rows - 2 >= h0 * (scale + 1):
                scale += 1
        else:                           # 窄终端退单字符
            while cols >= w0 * (scale + 1) and rows - 2 >= h0 * (scale + 1):
                scale += 1
    w, h = w0 * scale, h0 * scale       # 谐波是连续函数, 重建网格随便放大
    fps = fps or d["fps"]
    wide = cols >= w * 2                # 终端够宽才用双字符, 否则折行必抽搐
    blank, ink_ch = ("  ", "██") if wide else (" ", "#")
    out = sys.stdout
    if scale > 1:
        print(f"adaptive: {w0}x{h0} -> {w}x{h} grid, "
              f"{'double' if wide else 'single'} char", file=sys.stderr)
    q = queue.Queue(maxsize=8)

    def worker():                       # 后台预渲染, 播放循环只管节拍
        for fr in d["frames"]:
            coefs, inv = _frame_polys(fr)
            polys = [decode_contour(c) * scale for c in coefs]
            mask = fill_evenodd(polys, w, h)
            ink = ~mask if inv else mask
            q.put("\x1b[H" + "\n".join(
                "".join(ink_ch if v else blank for v in row) for row in ink))
        q.put(None)
    threading.Thread(target=worker, daemon=True).start()

    out.write("\x1b[?25l\x1b[2J")       # 藏光标, 只在开场清一次屏
    t0, i = time.perf_counter(), 0
    try:
        while True:
            buf = q.get()
            if buf is None:
                break
            out.write(buf)
            out.flush()
            i += 1
            pause = t0 + i / fps - time.perf_counter()
            if pause > 0:
                time.sleep(pause)
    finally:
        out.write("\x1b[?25h\n")

# ---------- 渲染成 mp4 (根治终端刷新率) ----------
def render(json_path, out_mp4, src_video=None, scale=8):
    import subprocess
    d = json.load(open(json_path))
    w, h, fps = d["w"], d["h"], d["fps"]
    vw, vh = w * scale, h * scale
    cmd = ["ffmpeg", "-y", "-v", "error",
           "-f", "rawvideo", "-pix_fmt", "gray",
           "-s", f"{w}x{h}", "-r", str(fps), "-i", "-"]
    if src_video:
        cmd += ["-i", src_video]
    cmd += (["-filter_complex",
             f"[0:v]scale={vw}:{vh}:flags=neighbor[v]"] if src_video
            else ["-vf", f"scale={vw}:{vh}:flags=neighbor"])
    if src_video:
        cmd += ["-map", "[v]", "-map", "1:a?"]
    cmd += ["-c:v", "libx264", "-preset", "medium", "-crf", "18",
            "-pix_fmt", "yuv420p", "-shortest", out_mp4]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for i, fr in enumerate(d["frames"]):
        coefs, inv = _frame_polys(fr)
        mask = fill_evenodd([decode_contour(c) for c in coefs], w, h)
        g = (mask if inv else ~mask) * 255     # 白底黑影 / 黑底白影
        proc.stdin.write(g.astype(np.uint8).tobytes())
        if (i + 1) % 500 == 0:
            print("render", i + 1, flush=True)
    proc.stdin.close(); proc.wait()
    print("saved", out_mp4)

# ---------- 单帧自检: 心形参数曲线 ----------
def demo(k=16):
    t = np.linspace(0, 2 * np.pi, 2000, endpoint=False)
    x = 16 * np.sin(t) ** 3
    y = 13 * np.cos(t) - 5 * np.cos(2 * t) - 2 * np.cos(3 * t) - np.cos(4 * t)
    poly = np.c_[28 + x * 1.5, 17 - y * 0.75]
    W, H = 56, 30
    m0 = fill_evenodd([poly], W, H)
    rec = decode_contour(poly_to_coef(poly, k=k), 1200)
    m1 = fill_evenodd([rec], W, H)
    iou = (m0 & m1).sum() / max(1, (m0 | m1).sum())
    print("\n".join("".join("#" if v else "." for v in row) for row in m1))
    print(f"\nK={k}  IoU={iou:.3f}")

if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "demo"
    if cmd == "encode":
        encode_video(sys.argv[2], sys.argv[3])
    elif cmd == "play":
        fps = scale = None
        argv = sys.argv
        if "--fps" in argv:
            fps = float(argv[argv.index("--fps") + 1])
        if "--scale" in argv:
            scale = int(argv[argv.index("--scale") + 1])
        play(argv[2], fps, scale)
    elif cmd == "render":
        # render out.json out.mp4 [src.mp4]
        render(sys.argv[2], sys.argv[3],
               sys.argv[4] if len(sys.argv) > 4 else None)
    else:
        demo()
