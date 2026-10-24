#!/usr/bin/env python3
# fourier_badapple.py — 傅里叶级数画笔, 终端播放 (极致性能版)
# 三级缓存: 系数 npz(免 json 解析) -> 选中帧折线 npz(免级数合成) -> E 矩阵常驻(免 exp)
# 生产者预烤字节帧, 写循环只做 os.write + 节拍休眠
# 用法:
#   python3 fourier_badapple.py badapple.json
#   可选: --hold N (每帧 N 个动画帧, 默认12)  --fps F  --cols C
#   --circles 演示傅里叶圆链本体 (默认只描级数曲线)  --no-cache 禁用缓存
import os, sys, time, json, queue, shutil, threading
import numpy as np

CH_S, CH_BG, CH_DOT, CH_HASH, CH_AT = 32, 58, 46, 35, 64  # ' ' ':' '.' '#' '@'
SAMPLES = 256
PREFIX = b"\x1b[H"
_E_CACHE = {}                                          # (k0,k1) -> exp 相位矩阵

def _frame_polys(fr):
    if isinstance(fr, dict):
        return fr.get("f", []), bool(fr.get("inv", False))
    return fr, False

# ---------- 光栅化 ----------
def _draw_ellipse(buf, cx, cy, rx, ry, ch):
    H, W = buf.shape
    ix, iy = int(round(cx)), int(round(cy))
    if rx < 0.8 and ry < 0.8:
        if 0 <= ix < W and 0 <= iy < H:
            buf[iy, ix] = ch
        return
    n = max(int(2 * np.pi * max(rx, ry)), 10)
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    xs = np.clip(np.round(cx + rx * np.cos(th)).astype(np.intp), 0, W - 1)
    ys = np.clip(np.round(cy + ry * np.sin(th)).astype(np.intp), 0, H - 1)
    buf[ys, xs] = ch

def _poly_points(pts, H, W):
    """闭合折线 -> 散点 (xs, ys) 与线段偏移 off, 全程向量化"""
    p0, p1 = pts[:-1], pts[1:]
    n = np.maximum(np.abs(p1 - p0).max(axis=1).astype(int) + 2, 2)
    off = np.concatenate([[0], np.cumsum(n)])
    gt = np.arange(int(off[-1]))
    sid = np.repeat(np.arange(len(n)), n)
    t = (gt - off[sid]) / n[sid]
    seg = p0[sid] + (p1 - p0)[sid] * t[:, None]
    xs = np.clip(np.round(seg[:, 0]).astype(np.intp), 0, W - 1)
    ys = np.clip(np.round(seg[:, 1]).astype(np.intp), 0, H - 1)
    return xs, ys, off

# ---------- 谐波 ----------
def _decode(k0, k1, n, C):
    key = (k0, k1)
    E = _E_CACHE.get(key)
    if E is None:
        t = np.arange(n) / n
        E = np.exp(2j * np.pi * np.outer(t, np.arange(k0, k1 + 1)))
        _E_CACHE[key] = E
    z = E @ (C[:, 0] + 1j * C[:, 1])
    return np.c_[z.real, z.imag]

def _area(pts):
    x, y = pts[:, 0], pts[:, 1]
    return abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) / 2

# ---------- 帧仓库 ----------
class FrameStore:
    """两级缓存: 系数 npz(与选帧无关) + 选中帧折线 npz(与 hold 绑定)"""
    def __init__(self, path, use_cache=True):
        self.w = self.h = self.fps = None
        self.src = path
        cp = os.path.splitext(path)[0] + ".cache.npz"
        if use_cache and os.path.exists(cp) and \
           (not os.path.exists(path) or os.path.getmtime(cp) >= os.path.getmtime(path)):
            z = np.load(cp)
            self.data, self.k0, self.k1 = z["data"], z["k0"], z["k1"]
            self.n, self.cf, self.hoff, self.inv = z["n"], z["cf"], z["hoff"], z["inv"]
            self.w, self.h, self.fps = (int(z["meta"][0]), int(z["meta"][1]),
                                        int(z["meta"][2]))
        else:
            self._load_json(path)
            if use_cache:
                try:
                    np.savez(cp, data=self.data, k0=self.k0, k1=self.k1, n=self.n,
                             cf=self.cf, hoff=self.hoff, inv=self.inv,
                             meta=np.array([self.w, self.h, self.fps], np.int64))
                except OSError:
                    pass
        self._pl = None                                     # 折线缓存

    def _load_json(self, path):
        d = json.load(open(path))                           # 一次性成本
        self.w, self.h, self.fps = d["w"], d["h"], d["fps"]
        data, k0, k1, nn = [], [], [], []
        cf, hoff, inv = [0], [0], []
        for fr in d["frames"]:
            coefs, iv = _frame_polys(fr)
            inv.append(iv)
            for c in coefs:
                C = np.asarray(c["c"], np.float32)
                data.append(C)
                k0.append(c["k"][0]); k1.append(c["k"][1]); nn.append(c["n"])
                hoff.append(hoff[-1] + len(C))
            cf.append(len(k0))
        self.data = np.concatenate(data) if data else np.zeros((0, 2), np.float32)
        self.k0 = np.array(k0, np.int32); self.k1 = np.array(k1, np.int32)
        self.n = np.array(nn, np.int32)
        self.cf, self.hoff = np.array(cf, np.int64), np.array(hoff, np.int64)
        self.inv = np.array(inv, bool)
        del d

    def __len__(self):
        return len(self.inv)

    def get(self, i):
        """原始系数 ( circles 模式仍需要 )"""
        c0, c1 = self.cf[i], self.cf[i + 1]
        h0 = self.hoff[c0]
        Cf = self.data[h0:self.hoff[c1]]
        coefs = [(int(self.k0[j]), int(self.k1[j]), int(self.n[j]),
                  Cf[self.hoff[j] - h0:self.hoff[j + 1] - h0]) for j in range(c0, c1)]
        return coefs, bool(self.inv[i])

    def ensure_polys(self, sel, use_cache=True, src=None):
        """为选中帧离线合成折线; src 为回退合成函数(i)->polys"""
        if not sel:
            return False
        pp = os.path.splitext(self.src)[0] + f".poly.{len(sel)}.npz"
        if use_cache and os.path.exists(pp):
            z = np.load(pp)
            if np.array_equal(z["sel"], np.asarray(sel, np.int64)):
                self._pl = (z["pl"], z["pcf"], z["poff"])
                return True
        pls, pcf, poff = [], [0], [0]
        for i in sel:
            for j in range(self.cf[i], self.cf[i + 1]):
                pts = src.decode_row(j) if src else self._decode_row(j)
                pls.append(pts.astype(np.float32))
                poff.append(poff[-1] + len(pts))
            pcf.append(len(poff) - 1)
        self._pl = (np.concatenate(pls) if pls else np.zeros((0, 2), np.float32),
                    np.array(pcf, np.int64), np.array(poff, np.int64))
        if use_cache:
            try:
                np.savez(pp, pl=self._pl[0], pcf=self._pl[1], poff=self._pl[2],
                         sel=np.asarray(sel, np.int64))
            except OSError:
                pass
        return True

    def _decode_row(self, j):
        h0, h1 = self.hoff[j], self.hoff[j + 1]
        return _decode(int(self.k0[j]), int(self.k1[j]), int(self.n[j]),
                       self.data[h0:h1])

    def polys_frame(self, s):
        """第 s 个选中帧的折线列表 (视图, 零拷贝)"""
        pl, pcf, poff = self._pl
        return [pl[poff[j]:poff[j + 1]] for j in range(pcf[s], pcf[s + 1])]

# ---------- 播放 ----------
def play(path, hold=12, fps=None, cols=None, circles=False, use_cache=True):
    store = FrameStore(path, use_cache)
    w0, h0 = store.w, store.h
    fps = fps or store.fps
    total = len(store)
    n_sel = max(1, total // hold)
    sel = [int(s * (total - 1) / max(1, n_sel - 1)) for s in range(n_sel)]
    has_pl = store.ensure_polys(sel, use_cache)
    tc = shutil.get_terminal_size((100, 30))
    W = int(min(2 * w0, (cols or tc.columns) - 1))
    H = max(int(round(W * h0 / (2.0 * w0))), 8)
    if H > tc.lines - 3:
        H = tc.lines - 3
        W = min(W, int(round(2.0 * H * w0 / h0)))
    sx, sy = (W - 1) / (w0 - 1), (H - 1) / (h0 - 1)
    n_chain = 2 if circles else 8
    print(f"fourier: {W}x{H}, {n_sel} frames x {hold}f, "
          f"poly-cache={'on' if has_pl else 'off'}", file=sys.stderr)
    q = queue.Queue(maxsize=240)
    canvas = np.empty((H, W + 1), np.uint8)
    canvas[:, W] = 10
    canvas[:, :W] = CH_S
    blank = PREFIX + canvas.tobytes()

    def _produce():
        for s in range(n_sel):
            if has_pl:
                polys = store.polys_frame(s)
                coefs = store.get(sel[s])[0] if circles else None
            else:
                coefs, _ = store.get(sel[s])
                polys = [_decode(*c) for c in coefs]
            trail = np.full((H, W), CH_S, np.uint8)
            for p in polys:
                xs, ys, _ = _poly_points(p * [sx, sy], H, W)
                trail[ys, xs] = CH_BG
            order = sorted(range(len(polys)), key=lambda k: -_area(polys[k]))
            chains = []
            for k in order[:n_chain]:
                pts = polys[k] * [sx, sy]
                pts = np.vstack([pts, pts[:1]])
                xs, ys, off = _poly_points(pts, H, W)
                nseg = len(off) - 1
                bounds = (nseg * np.arange(hold + 1)) // hold
                rings = None
                if circles and coefs:
                    a0, a1, _, C = coefs[k]
                    ks = np.arange(a0, a1 + 1)
                    amp = np.abs(C[:, 0] + 1j * C[:, 1])
                    io = np.argsort(-amp)
                    Cc = (C[:, 0] + 1j * C[:, 1])[io]; ks = ks[io]
                    ph = 2j * np.pi * np.outer(np.arange(hold) / hold, ks)
                    cent = Cc[0] + (Cc[1:] * np.exp(ph)).cumsum(axis=1)
                    rings = (np.real(cent) * sx, np.imag(cent) * sy, amp[io][1:])
                chains.append((xs, ys, off, bounds, rings))
            for j in range(hold):
                canvas[:, :W] = trail
                for xs, ys, off, bounds, rings in chains:
                    a, b = bounds[j], bounds[j + 1]
                    lo, hi = off[a], off[b]
                    if hi > lo:
                        trail[ys[lo:hi], xs[lo:hi]] = CH_HASH
                        canvas[:, :W] = trail
                        canvas[ys[hi - 1], xs[hi - 1]] = CH_AT
                    if rings is not None:
                        for r in range(rings[2].size):
                            if rings[2][r] < 0.3:
                                continue
                            _draw_ellipse(canvas, rings[0][j, r], rings[1][j, r],
                                          rings[2][r] * sx, rings[2][r] * sy, CH_DOT)
                q.put(PREFIX + canvas.tobytes())
        q.put(None)

    def worker():
        try:
            _produce()
        finally:
            q.put(None)
    threading.Thread(target=worker, daemon=True).start()

    out = sys.stdout.buffer
    out.write(b"\x1b[?25l\x1b[2J")
    beat, next_t = 1.0 / fps, time.perf_counter()
    try:
        while True:
            b = q.get()
            if b is None:
                break
            out.write(b)
            out.flush()
            next_t += beat
            lag = next_t - time.perf_counter()
            if lag > 0:
                time.sleep(lag)
            else:
                next_t = time.perf_counter()            # 落后不追帧, 防雪崩
    finally:
        out.write(b"\x1b[?25h\n")
        out.flush()

if __name__ == "__main__":
    if os.name == "nt":
        os.system("")
    if len(sys.argv) < 2:
        sys.exit("usage: fourier_badapple.py badapple.json [--hold N] [--fps F] [--cols C] [--circles] [--no-cache]")
    argv, fps, hold, cols, circles, cache = sys.argv, None, 12, None, False, True
    if "--fps" in argv: fps = float(argv[argv.index("--fps") + 1])
    if "--hold" in argv: hold = int(argv[argv.index("--hold") + 1])
    if "--cols" in argv: cols = int(argv[argv.index("--cols") + 1])
    if "--circles" in argv: circles = True
    if "--no-cache" in argv: cache = False
    play(argv[1], hold=hold, fps=fps, cols=cols, circles=circles, use_cache=cache)
