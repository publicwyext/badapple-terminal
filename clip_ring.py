#!/usr/bin/env python3
# clip_ring.py — 一次性手术: 从系数 JSON 里剜掉舞台相框环
# 视频自带相框 (左 12 列 / 右 84 列起 / 上 2 行), 随黑白反转换边, 总有一头落进墨里:
#   - 相框单独成环 -> 两侧神秘竖条
#   - 人物贴框时与相框焊成一条环 -> 贴边帧边框复活
# 手术: 受影响帧反演墨图 (4x 超采样栅格化) -> 抹掉环区 -> 重追环 -> 重编码系数
# 未受影响帧保持原系数不动 (零损伤)
import json, sys
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, '/root/badapple-terminal')
from encode_planb import trace_loops, poly_to_coef

SS = 4                 # 栅格化超采样
L_BAR, R_BAR, T_BAR = 13, 83, 3     # 相框内沿+1px 内唇晕边 (列/列/行)
KS = np.arange(-24, 25); IDX = KS % 256

def loop_pts(c):
    A = np.zeros(256, complex)
    cc = np.array(c['c']).ravel()
    A[IDX] = cc[0::2] + 1j * cc[1::2]
    z = np.fft.ifft(A) * 256
    return np.stack([z.real, z.imag], 1)

def main(src, dst):
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
        img = Image.new('L', (96 * SS, 48 * SS), 0)
        dr = ImageDraw.Draw(img)
        for p in polys:
            dr.polygon([((x + 0.5) * SS, (y + 0.5) * SS) for x, y in p], fill=255)
        ink = np.array(img.resize((96, 48), Image.BOX), float) >= 127.5
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

if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
