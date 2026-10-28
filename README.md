# badapple-terminal

**Bad Apple!!** in your terminal — outlines re-synthesized by truncated **Fourier series (epicycles)**, rasterized and played by a tiny C renderer with **zero runtime dependencies**.

**Bring your own video**: any mp4 you feed the pipeline becomes your own epicycle blob. Coefficients ship as a **BAP3 container**: quantized to int16 fixed-point (Q8) and sparsified (sub-0.125px harmonics zeroed), contours **matched frame-to-frame by centroid** and **phase-aligned** (start-point + orientation via FFT correlation) before slot-keyed differencing, split into independent chunks, each compressed with **xz** (data check NONE). Linked straight into the ELF via `ld -r -b binary` — no file I/O, no parsing at startup. Contours are additionally **purged of the stage's picture-frame**: the decorative frame ring (13 columns left, from column 83 right, 3 rows top at 96×48) rides whichever side of Otsu the B/W inversions put it on — spawning edge bars, and **fusing with the silhouette on contact**. It is masked out at the ink level (`encode_planb.py --sanitize src.json dst.json 13,83,3`; the silhouette is clipped flat at the frame edge, exactly like the original occlusion), deleting ~6,000 junk contours across 5,270 frames. A pool of **multicore streaming decoder threads** (vendored **XZ Embedded**, 0BSD) inflates chunks ahead of the render pipeline, so cold start → first frame stays **~17 ms**.

## Build

Requires: `gcc`, `python3` + `numpy`, `ffmpeg` in PATH.

```sh
make gen VIDEO=my.mp4    # video -> contours -> BAP3 blob -> link & compile
make                     # rebuild later without re-encoding
```

The one-liner runs the full pipeline — `encode_planb.py` (ffmpeg raw gray → Otsu → pixel-crack contour tracing → arc-length resample → FFT truncated to k ∈ [-24, 24]) then `make_blob.py` (quantize + inter-frame diff + chunked xz, decoder-mirror self-verified) — then embeds the blob and builds with `-O3 -flto + --gc-sections + strip`.

### Windows

Windows 10 1809+ (ANSI/VT console required). `make win` cross-compiles a fully static `badapple.exe` — only `KERNEL32.dll` + `msvcrt.dll` imports, nothing to install; the target verifies the artifact is really `PE32+ x86-64` and copies it to `badapple-windows-x86_64.exe`.

```sh
make win                 # needs gcc-mingw-w64-x86-64 (apt) or MSYS2 mingw-w64-x86_64-gcc
```

Port notes: clocks/pthreads come from winpthreads (`clock_gettime`/`clock_nanosleep` keep POSIX semantics); `win_compat.h` supplies VT enabling, console size (`GetConsoleScreenBufferInfo` instead of `TIOCGWINSZ`), a per-frame resize poll (Windows has no SIGWINCH) and `write`/`usleep`/`sysconf` shims. Resize is honored every frame instead of via signal.

## Usage

```sh
./badapple                  # play (30 fps, auto-fit to your window)
./badapple --fps 60 --threads 8 --dthreads 4
./badapple --cols 120       # force canvas width
./badapple --test 500       # render a single frame and exit
./badapple --selftest       # decode all chunks serially, print checksum
```

## Features

- **Differential ANSI rendering** — only changed cells are rewritten between frames, no full-screen repaints.
- **SIGWINCH adaptive canvas** — debounced (150 ms) rebuild while dragging; workers swap generations and the pipeline drains cleanly.
- **Pipelined pre-render** — N render workers bake frames ahead in strict order; the main thread only diffs cells and paces the beat.
- **Tabulated series synthesis** — the epicycle sum touches zero libm trig calls: all `e^{2πihs/256}` phases are 256th roots of unity, pre-baked once from a 64-entry quadrant-folded table into a 129×256 cos/sin LUT (~264 KB); `decode()` reduces to sequential SIMD FMA loops. Full-feature playback 3.25 s → 0.39 s (8.3×, threads=4), pixel-identical output, cold start 11.5 → 8.6 ms.
- **Chunked streaming decompression** — the BAP3 blob decodes lazily: D decoder threads claim chunks in order (bounded by a small slot pool), xz-inflate each independently, undo the frame-to-frame coefficient diffs, and publish decoded frames into the pipeline. Memory stays near the compressed size + a handful of chunk arenas.
- Self-contained binary, plain `#` ASCII output — works over any dumb SSH session.

## Pipeline notes

Frames flow through a 64-slot ring; chunks through a 6-slot decode pool. Decoder threads claim chunks **strictly in order and only while holding a free slot** (this pairing is what keeps an in-flight window contiguous and the pool deadlock-free); render workers wait on per-chunk readiness plus per-slot retirement. Slot retirement is judged **by lap** (`rd >= job - 63`), not by comparing stale slot markers — the marker-comparison form admits an out-of-order lap that stomps the marker the main thread is waiting on (a latent race the v1 renderer carried; it surfaced once decode threading changed the timing).

On resize, stale-size frames are dropped by advancing the read cursor; chunks fully below the cursor are reclaimed, half-decoded stragglers are recycled on publish.

## BAP3 container

```
'BAP3' u32 W H FPS TOTAL NCHUNK CHUNK_FRAMES
NCHUNK × { u32 comp_off comp_len raw_len frame_first }   # comp_off from blob start
NCHUNK independent xz streams (LZMA2 9e, 2 MiB dict, data check NONE) concatenated
[optional audio tail] MP3 bytes + u32 le mp3_len + 'BAUD'
```

The optional audio tail is appended by `make gen` (ffmpeg → mono 16 kHz 16 kbps MP3 ≈ 429 KB for the 3'39" song). The player locates it from the end marker, starts playback on the same `CLOCK_MONOTONIC` beat as the first frame, and falls back to silent if there is no tail, no sound device, or the build is `make NO_AUDIO=1`. Decode/playback is vendored single-header `miniaudio.h` (dr_mp3 + device I/O, zero external deps); `--test` never opens a device.


Per frame: `u16 (cnt | 0x8000=KEY)`, then per contour: `u8 (k0+64 | 0x80=ABS) u8 (k1+64) u8 slot` and int16 pairs. `slot` names a reference slot (0..254): if the slot is live and the record is not ABS, the int16s are a **residual**; otherwise they are absolute values — either way the decoded result is written back into the slot. Slots are keyed by **centroid nearest-neighbor matching + FFT phase/orientation alignment**, so contour reordering between frames no longer poisons residuals. Chunks stay self-contained (first frame is always KEY), so chunks decode in parallel and `--test` touches only the chunk it needs. Quantization is Q8 (×256); harmonics below 0.125 px are zeroed (an order of magnitude below the 0.54 px FFT-truncation error) — the sparsity is what shrinks the blob ~54 % versus BAP2.

`make_blob.py` verifies every chunk by running a decoder mirror over the emitted bytes and asserting value-exact reconstruction before writing anything.

## Regenerate the blob, step by step

`make gen VIDEO=my.mp4` already does all of this; the manual path, if you want to poke at intermediate artifacts:

```sh
# 1) video -> per-frame closed contours, Fourier-truncated to k in [-24, 24]
python3 encode_planb.py my.mp4 badapple.json

# 2) JSON -> BAP3 (sparsify + centroid/phase match + slot diff + chunked xz, self-verified)
python3 make_blob.py badapple.json badapple_blob.bin

# 3) embed & build (or just: make)
ld -r -b binary -o blob.o badapple_blob.bin
gcc -O3 -march=native -flto -fno-plt -fomit-frame-pointer -ffunction-sections -fdata-sections badapple.c xz_embedded/*.c blob.o -o badapple -lm -lpthread -Wl,--gc-sections,--build-id=none
strip --strip-all -R .comment -R .note -R .note.gnu.build-id badapple
```

Note: ~95% of the binary is the already-xz-compressed blob; do NOT UPX-pack the final binary — it saves ~47 KB but doubles startup (11.3 ms → 20.8 ms measured).

How it works: ffmpeg pumps raw gray frames → Otsu threshold → **pixel-crack contour tracing** (closed loops, saddle-point safe) → arc-length resampling to 256 points → `np.fft` truncated to 49 harmonics per contour. Playback resynthesizes each outline with `z(t) = Σ c_k e^{2πikt}` — that's the epicycle.

Repo files: `badapple.c` (renderer + streaming decoder) · `encode_planb.py` / `make_blob.py` (generation pipeline) · `Makefile` (BYO video one-liner) · `xz_embedded/` (vendored XZ Embedded, 0BSD). No pre-baked blob ships here — encode your own.
