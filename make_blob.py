#!/usr/bin/env python3
# make_blob.py — 系数 JSON -> 紧凑二进制 blob (供 badapple.c 经 ld -r -b binary 嵌入)
#
# blob 布局 (全部小端):
#   [0] u32 W0      [1] u32 H0      [2] u32 FPS     [3] u32 TOTAL
#   之后每帧: u32 轮廓数 cnt
#     每轮廓: i32 k0, i32 k1, u32 n, float32 c[2*(k1-k0+1)]  (交错 re,im)
#
# 用法:  python3 make_blob.py badapple.json badapple_blob.bin
# 嵌入:  ld -r -b binary -o blob.o badapple_blob.bin
#        gcc -O2 -march=native badapple.c blob.o -o badapple -lm -lpthread
import json, struct, sys

def make_blob(in_json, out_bin):
    d = json.load(open(in_json))
    out = bytearray(struct.pack('<4I', d['w'], d['h'], d['fps'], len(d['frames'])))
    for fr in d['frames']:
        cs = fr.get('f') or []
        out += struct.pack('<I', len(cs))
        for c in cs:
            k0, k1 = c['k']
            flat = [v for pair in c['c'] for v in pair]
            assert len(flat) == 2 * (k1 - k0 + 1), f"coefficient count mismatch: {len(flat)}"
            out += struct.pack('<iiI', k0, k1, c['n'])
            out += struct.pack(f'<{len(flat)}f', *flat)
    open(out_bin, 'wb').write(out)
    print(f'{out_bin}: {len(out)} bytes, {len(d["frames"])} frames')

if __name__ == '__main__':
    make_blob(sys.argv[1] if len(sys.argv) > 1 else 'badapple.json',
              sys.argv[2] if len(sys.argv) > 2 else 'badapple_blob.bin')
