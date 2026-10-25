/* xz_config.h — userspace shim for XZ Embedded (kernel-tree layout)
 * Vendored from https://github.com/tukaani-project/xz-embedded (0BSD).
 * Maps the handful of kernel primitives the decoder needs onto libc. */
#ifndef XZ_CONFIG_H
#define XZ_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "xz.h"

#define GFP_KERNEL 0
static inline void *kmalloc(size_t n, int gfp) { (void)gfp; return malloc(n); }
static inline void *vzalloc(size_t n) { return calloc(1, n); }
static inline void kfree(void *p) { free(p); }
static inline void vfree(void *p) { free(p); }

#define memeq(a, b, size) (memcmp(a, b, size) == 0)
#define memzero(buf, size) memset(buf, 0, size)
#define min(a, b) ((a) < (b) ? (a) : (b))
#define min_t(type, a, b) ((type)(a) < (type)(b) ? (type)(a) : (type)(b))

static inline void *vmalloc(size_t n) { return malloc(n); }

#ifndef fallthrough
#define fallthrough __attribute__((fallthrough))
#endif

static inline uint32_t get_le32(const uint8_t *p) {
	uint32_t v;
	memcpy(&v, p, 4);
	return v; /* little-endian host; byteswap here if you port to BE */
}

#endif
