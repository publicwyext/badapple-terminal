# badapple-terminal

**Bad Apple!!** in your terminal — outlines re-synthesized by truncated **Fourier series (epicycles)**, rasterized and played by a tiny C renderer with **zero runtime dependencies**.

The coefficient blob (~7.4 MB) is linked straight into the ELF via `ld -r -b binary` — no parsing, no file I/O at startup. Cold start → first frame: **~17 ms**.

## Build

```sh
ld -r -b binary -o blob.o badapple_blob.bin
gcc -O2 -march=native badapple.c blob.o -o badapple -lm -lpthread
```

## Usage

```sh
./badapple             # play (30 fps, auto-fit to your window)
./badapple --fps 60 --threads 8
./badapple --cols 120  # force canvas width
./badapple --test 500  # render a single frame and exit
```

## Features

- **Differential ANSI rendering** — only changed cells are rewritten between frames (`ESC[row;colH` + incremental cells), no full-screen repaints.
- **SIGWINCH adaptive canvas** — debounced (150 ms) rebuild while dragging; workers swap generations and the pipeline drains cleanly.
- **Pipelined pre-render** — N worker threads bake frames ahead in strict order; the main thread only diffs cells and paces the beat.
- Self-contained binary, plain `#` ASCII output — works over any dumb SSH session.

## Pipeline notes

Frames flow through a 64-slot ring. Workers wait on per-slot sequence numbers; the main thread consumes in order against a monotonic clock (no frame-rate avalanche). On resize, stale-size frames are dropped by advancing the read cursor to the job cursor — nothing half-rendered ever reaches the terminal.
