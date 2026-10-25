/* badapple.c — 傅里叶级数画笔, 终端播放 (C 重写·极致性能版)
 * 数据嵌入: BAP3 容器 (int16 量化+稀疏化, 质心匹配槽位差分, 分块 xz/CHECK_NONE)
 *           由 ld -r -b binary 连入
 * 解压: 内嵌 XZ Embedded (0BSD), 按块流式解码, 多核解码线程池, 块间零依赖可并行;
 *       refs/raw/xz 实例全部线程本地复用, 稳态解码零分配
 * 渲染: 只用 '#' (ASCII), 画布每格 1 字节
 * 并行: 默认主线程内联渲染 (单帧仅几十微秒); 画布足够宽 (>= WORKER_COLS 列) 或显式
 *       --threads N>1 时才启用工人预烤流水线。线程全部 cond 等待, 无轮询。
 * 差分: 帧间只输出变化的格子; 整帧拼进单缓冲, 一次 write() 落盘 (EINTR/部分写安全);
 *       光标定位手写 itoa; 同行相邻变化段间隔 < 6 格时并入前段 (定位序列本身 6~10 字节)
 * 光栅: 16.16 定点 DDA, 每段一次除法, 逐步零 lround
 * 用法: ./badapple          <-- 数据已在体内 (make gen VIDEO=my.mp4 自带)
 *   可选: --fps F  --threads N  --dthreads N  --cols C  --test K  --selftest
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>
#include <pthread.h>
#include <stdatomic.h>
#include <time.h>
#include <signal.h>
#include <sys/ioctl.h>
#include "xz_embedded/xz.h"

extern const unsigned char _binary_badapple_blob_bin_start[];
extern const unsigned char _binary_badapple_blob_bin_end[];

#define SAMPLES 256
#define NSLOT 64
#define BLK '#'
static const char *g_fg = "\xe2\x96\x88";  /* █ 白块, --char 可换 */
#define QSCALE (1.0 / 256.0)          /* BAP3: int16 定点, Q8 */
#define MAXCOEF 98                    /* 2*(24+24+1) */
#define NREF 255                      /* 差分参考槽, u8 槽号, 0xFF=绝对不占槽 */
#define NCSLOT 6                      /* 解码块槽位池 */
#define MERGE_GAP 6                   /* 同行变化段间隔小于此格数则并入前段 */
#define WORKER_COLS 240               /* 画布达到此宽度才默认启用预烤工人 */

static uint32_t W0, H0, FPS, TOTAL;   /* BAP2 头 */
static uint32_t NCHUNK, CHUNKF;
static uint32_t W, H;                 /* 画布尺寸 */
static double SX, SY;
static int NTHREAD = 0;               /* 0 = 自动: 宽画布才开工人 */
static int NDEC = 0;                  /* 解码线程, 0=自动 */
static int g_inline = 1;              /* 主线程内联渲染 */
static size_t TICK_SZ;
static volatile sig_atomic_t g_stop = 0;

typedef struct { int k0, k1; const float *c; } Contour;
typedef struct { Contour *ct; int nct; } Frame;

static const uint8_t *g_blob;
static size_t g_blob_len;
typedef struct { uint32_t off, clen, rlen, first; } ChunkEnt;
static ChunkEnt *g_chk;

/* ---------- 解码块槽位池 ---------- */
typedef struct {
    int state;                        /* 0 空, 1 解压中, 2 就绪 */
    long chunk;
    uint32_t nfr;                     /* 本块实际帧数 */
    Frame *fr;                        /* [nfr] */
    Contour *ct;
    float *fc;
} CSlot;
static CSlot cpool[NCSLOT];
static long *cslot_of;                /* [NCHUNK] 0=无, 否则槽位+1 */
static long cur_chunk = 0;            /* 主线程读游标所在块 */
static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cv = PTHREAD_COND_INITIALIZER;

static void free_slot_arrays(CSlot *cs) {
    free(cs->fr); free(cs->ct); free(cs->fc);
    cs->fr = NULL; cs->ct = NULL; cs->fc = NULL;
}

static inline Frame *frame_of(long fi) {
    long c = fi / CHUNKF;
    return &cpool[cslot_of[c] - 1].fr[fi - c * CHUNKF];
}

/* ---------- 级数合成: 预烤相位表, 内层纯 FMA 可向量化 ----------
 * 相位 2*pi*h*s/SAMPLES 是 256 次单位根格点: e^{i2pihs/256} = W[(h*s)&255],
 * W 只需 64 个独立值 + 象限镜像; 全表 CTab/STab[129][256] 由其展开, 约 264KB,
 * 一次烤好后 decode 内层为顺序扫描的 FMA 循环, 无任何 libm 三角函数调用。 */
static float CTab[129][SAMPLES], STab[129][SAMPLES];

static void tables_init(void) {
    static float Wc[SAMPLES], Ws[SAMPLES];
    for (int m = 0; m < SAMPLES / 4; m++) {
        double th = 2.0 * M_PI * m / SAMPLES;
        Wc[m] = (float)cos(th); Ws[m] = (float)sin(th);
    }
    for (int m = SAMPLES / 4; m < SAMPLES; m++) {
        int q = m & (SAMPLES / 4 - 1);
        switch (m >> 6) {
        case 1: Wc[m] = -Ws[q]; Ws[m] =  Wc[q]; break;
        case 2: Wc[m] = -Wc[q]; Ws[m] = -Ws[q]; break;
        case 3: Wc[m] =  Ws[q]; Ws[m] = -Wc[q]; break;
        }
    }
    for (int h = -64; h <= 64; h++)
        for (int s = 0; s < SAMPLES; s++) {
            int m = (h * s) & (SAMPLES - 1);
            CTab[h + 64][s] = Wc[m];
            STab[h + 64][s] = Ws[m];
        }
}

/* h = k0+j 取值 [-64,63], 索引 h+64 ∈ [0,128] 恰好铺满全表 */
static void decode(const Contour *ct, double *pts) {
    int K = ct->k1 - ct->k0 + 1;
    float zr[SAMPLES], zi[SAMPLES];
    for (int s = 0; s < SAMPLES; s++) zr[s] = ct->c[0], zi[s] = ct->c[1];
    for (int j = 1; j < K; j++) {
        const float *cs = CTab[ct->k0 + j + 64], *sn = STab[ct->k0 + j + 64];
        const float cr = ct->c[2 * j], ci = ct->c[2 * j + 1];
        for (int s = 0; s < SAMPLES; s++) {
            zr[s] += cr * cs[s] - ci * sn[s];
            zi[s] += cr * sn[s] + ci * cs[s];
        }
    }
    for (int s = 0; s < SAMPLES; s++) { pts[2 * s] = zr[s]; pts[2 * s + 1] = zi[s]; }
}

/* ---------- 光栅化: 16.16 定点 DDA, 每段一次除法, 逐步零 lround ---------- */
static inline void plot(unsigned char *buf, int x, int y) {
    if (x < 0 || x >= (int)W || y < 0 || y >= (int)H) return;
    buf[(size_t)y * (W + 1) + x] = BLK;
}

static void plot_all(const double *pts, int npts, unsigned char *buf) {
    int m = npts;
    for (int s = 0; s < m; s++) {
        double x0 = pts[2*s] * SX,        y0 = pts[2*s+1] * SY;
        double x1 = pts[2*((s+1)%m)] * SX, y1 = pts[2*((s+1)%m)+1] * SY;
        double dx = x1 - x0, dy = y1 - y0;
        double ax = fabs(dx), ay = fabs(dy);
        int steps = (int)(ax > ay ? ax : ay) + 2;
        int64_t xi = llround(x0 * 65536.0), yi = llround(y0 * 65536.0);
        int64_t xs = llround(dx * 65536.0 / (double)steps);
        int64_t ys = llround(dy * 65536.0 / (double)steps);
        for (int i = 0; i < steps; i++) {
            plot(buf, (int)((xi + 32768) >> 16), (int)((yi + 32768) >> 16));
            xi += xs; yi += ys;
        }
    }
}

static void blank_tick(unsigned char *t) {
    t[0] = 0x1b; t[1] = '['; t[2] = 'H';
    unsigned char *p = t + 3;
    for (uint32_t r = 0; r < H; r++) {
        memset(p, ' ', W); p += W; *p++ = '\n';
    }
}

static void render_frame(long fi, unsigned char *t) {
    blank_tick(t);
    Frame *fr = frame_of(fi);
    if (fr->nct <= 0) return;
    double tmp[2*SAMPLES];
    for (int k = 0; k < fr->nct; k++) {
        decode(&fr->ct[k], tmp);
        plot_all(tmp, SAMPLES, t);
    }
}

/* ---------- 差分输出: 整帧单缓冲, 一次 write ---------- */
static unsigned char *g_prev = NULL;
static char *emit_buf = NULL;
static size_t emit_cap = 0;

static void emit_buf_reserve(void) {
    size_t need = (size_t)H * (W * 3 + 64) + 64;
    if (emit_cap < need) {
        free(emit_buf);
        emit_cap = need + need / 2;
        emit_buf = malloc(emit_cap);
        if (!emit_buf) { fprintf(stderr, "badapple-c: oom\n"); exit(1); }
    }
}

/* SIGWINCH 会打断 write: EINTR 重试, 部分写续传, 绝不输出半截转义序列 */
static int full_write(int fd, const void *buf, size_t n) {
    const char *p = buf;
    while (n) {
        ssize_t w = write(fd, p, n);
        if (w < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        p += w; n -= (size_t)w;
    }
    return 0;
}

static inline char *put_u(char *p, uint32_t v) {
    char t[10]; int n = 0;
    do { t[n++] = (char)('0' + v % 10u); v /= 10u; } while (v);
    while (n) *p++ = t[--n];
    return p;
}

static inline char *put_cells(char *p, const unsigned char *s, uint32_t n) {
    while (n--) {
        if (*s == BLK) { for (const char *q = g_fg; *q; q++) *p++ = *q; s++; }
        else *p++ = (char)*s++;
    }
    return p;
}

static void emit_frame(int fd, const unsigned char *cur) {
    char *p = emit_buf;
    if (!g_prev) {
        g_prev = malloc((size_t)H * (W + 1));
        if (!g_prev) { fprintf(stderr, "badapple-c: oom\n"); exit(1); }
        memset(g_prev, ' ', (size_t)H * (W + 1));
        memcpy(p, "\x1b[2J", 4); p += 4;
        for (uint32_t r = 0; r < H; r++) {
            memcpy(p, "\x1b[", 2); p += 2;
            p = put_u(p, r + 1); *p++ = ';'; *p++ = '1'; *p++ = 'H';
            p = put_cells(p, cur + (size_t)r * (W + 1), W);
        }
    } else {
        for (uint32_t r = 0; r < H; r++) {
            const unsigned char *a = g_prev + (size_t)r * (W + 1);
            const unsigned char *b = cur   + (size_t)r * (W + 1);
            uint32_t c = 0;
            while (c < W) {
                while (c < W && a[c] == b[c]) c++;
                if (c >= W) break;
                uint32_t c0 = c, c1;
                for (;;) {
                    while (c < W && a[c] != b[c]) c++;
                    c1 = c;
                    uint32_t gap0 = c;
                    while (c < W && a[c] == b[c]) c++;
                    if (c < W && c - gap0 < MERGE_GAP) continue;  /* 小间隙并入前段 */
                    break;
                }
                memcpy(p, "\x1b[", 2); p += 2;
                p = put_u(p, r + 1); *p++ = ';';
                p = put_u(p, c0 + 1); *p++ = 'H';
                p = put_cells(p, b + c0, c1 - c0);
            }
        }
    }
    memcpy(p, "\x1b[H", 3); p += 3;
    if (full_write(fd, emit_buf, (size_t)(p - emit_buf)) != 0) g_stop = 1;
    memcpy(g_prev, cur, (size_t)H * (W + 1));
}

/* ---------- BAP3: xz 解压 + 反差分 (质心匹配槽位参考, 稀疏化系数量化) ---------- */
typedef struct { int32_t v[MAXCOEF]; int valid; } RefSlot;

static uint32_t get_u32(const uint8_t *p) { uint32_t v; memcpy(&v, p, 4); return v; }

/* xz_dec 单次调用模式自带 reset, 解码器实例可跨块复用 (线程内串行) */
static int xz_chunk(const ChunkEnt *e, uint8_t *out, struct xz_dec *dec) {
    struct xz_buf b = {
        .in = g_blob + e->off, .in_pos = 0, .in_size = e->clen,
        .out = out, .out_pos = 0, .out_size = e->rlen,
    };
    enum xz_ret r = xz_dec_run(dec, &b);
    return r == XZ_STREAM_END ? 0 : -1;
}

/* raw 块字节 -> 槽位内 Frame/Contour/float 系数; 返回 0 成功 */
static int undiff_chunk(const uint8_t *raw, size_t rlen, CSlot *cs, RefSlot *refs) {
    size_t pos = 0;
    uint32_t nframes = 0, nct = 0;
    /* pass1: 计数 */
    while (pos < rlen) {
        uint16_t hdr; memcpy(&hdr, raw + pos, 2); pos += 2;
        nframes++;
        nct += hdr & 0x7FFF;
        for (uint32_t k = 0; k < (uint32_t)(hdr & 0x7FFF); k++) {
            if (pos + 3 > rlen) return -1;
            int k0 = (raw[pos] & 0x7F) - 64, k1 = raw[pos + 1] - 64;
            pos += 3 + 2u * (uint32_t)(2 * (k1 - k0 + 1));
        }
    }
    if (pos != rlen || nframes > CHUNKF) return -1;
    cs->nfr = nframes;
    cs->fr = malloc(sizeof(Frame) * nframes);
    cs->ct = malloc(sizeof(Contour) * nct);
    cs->fc = malloc(sizeof(float) * nct * MAXCOEF);
    if (!cs->fr || !cs->ct || !cs->fc) return -1;
    /* pass2: 反差分 -> float; 槽号即参考, 与帧内序号无关 */
    pos = 0; uint32_t fb = 0, cb = 0;
    for (uint32_t f = 0; f < nframes; f++) {
        uint16_t hdr; memcpy(&hdr, raw + pos, 2); pos += 2;
        if (hdr & 0x8000)
            for (int s = 0; s < NREF; s++) refs[s].valid = 0;
        int cnt = hdr & 0x7FFF;
        cs->fr[f].ct = &cs->ct[cb];
        cs->fr[f].nct = cnt;
        for (int j = 0; j < cnt; j++) {
            int k0 = (raw[pos] & 0x7F) - 64, k1 = raw[pos + 1] - 64;
            uint8_t abs_flag = raw[pos] & 0x80;
            uint8_t slot = raw[pos + 2];
            pos += 3;
            if (slot != 0xFF && slot >= NREF) return -1;
            int n = 2 * (k1 - k0 + 1);
            RefSlot *rf = slot != 0xFF ? &refs[slot] : NULL;
            int use_ref = rf && rf->valid && !abs_flag;
            float *out = &cs->fc[(size_t)cb * MAXCOEF];
            for (int q = 0; q < n; q++) {
                int16_t r; memcpy(&r, raw + pos + 2*q, 2);
                int32_t v = use_ref ? rf->v[q] + r : r;
                if (rf) rf->v[q] = v;
                out[q] = (float)(v * QSCALE);
            }
            pos += 2u * n;
            if (rf) rf->valid = 1;
            cs->ct[cb].k0 = k0; cs->ct[cb].k1 = k1; cs->ct[cb].c = out;
            cb++;
        }
        fb++;
    }
    return (fb == nframes && cb == nct) ? 0 : -1;
}

/* 解一块: 连号领取(先占槽后领号, 保证在飞块连续) -> xz -> undiff -> 就绪 */
static long last_claimed = -1;
static void decode_chunk_body(long c, CSlot *cs);

static void *decoder(void *arg) {
    (void)arg;
    for (;;) {
        if (g_stop) break;
        pthread_mutex_lock(&mu);
        int s = -1;
        long c = -1;
        while (!g_stop) {
            if (last_claimed + 1 >= (long)NCHUNK) break;   /* 已被同伴领完 */
            for (int i = 0; i < NCSLOT; i++)
                if (cpool[i].state == 0) { s = i; break; }
            if (s >= 0) { c = last_claimed + 1; last_claimed = c; break; }
            pthread_cond_wait(&cv, &mu);   /* 谓词每轮重算; 唤醒源: 槽释放/块完成/停止 */
        }
        if (c < 0 || s < 0) { pthread_mutex_unlock(&mu); break; }
        cpool[s].state = 1; cpool[s].chunk = c;
        cslot_of[c] = s + 1;
        pthread_mutex_unlock(&mu);
        decode_chunk_body(c, &cpool[s]);
    }
    return NULL;
}

/* 线程本地刮擦区: refs(~200KB)/raw/xz 实例全复用, 稳态解码零 malloc */
static __thread RefSlot *t_refs;
static __thread uint8_t *t_raw;
static __thread size_t t_rawcap;
static __thread struct xz_dec *t_xz;

static void decode_chunk_body(long c, CSlot *cs) {
    int fail = 0;
    if (c < cur_chunk) {                       /* resize 跳过的块: 白解, 直接回收 */
        fail = 1;
    } else {
        if (!t_refs) {
            t_refs = calloc(NREF, sizeof(RefSlot));
            if (!t_refs) fail = 1;
        }
        if (!fail && t_rawcap < g_chk[c].rlen) {
            free(t_raw);
            t_rawcap = g_chk[c].rlen + g_chk[c].rlen / 4;
            t_raw = malloc(t_rawcap);
        }
        if (!fail && !t_raw) fail = 1;
        if (!fail && !t_xz) t_xz = xz_dec_init(XZ_SINGLE, 0);
        if (!fail) {
            fail = !t_xz || xz_chunk(&g_chk[c], t_raw, t_xz) != 0 ||
                   undiff_chunk(t_raw, (size_t)g_chk[c].rlen, cs, t_refs) != 0;
        }
    }
    pthread_mutex_lock(&mu);
    if (fail) {
        fprintf(stderr, "badapple-c: chunk %ld decode failed\n", c);
        g_stop = 1;
        free_slot_arrays(cs);
        cslot_of[c] = 0;
        cs->state = 0;
    } else if (c < cur_chunk) {
        cslot_of[c] = 0;
        cs->state = 0;
        free_slot_arrays(cs);
    } else {
        cs->state = 2;
    }
    pthread_cond_broadcast(&cv);
    pthread_mutex_unlock(&mu);
}

/* ---------- 流水线: 工人按序预烤, 主线程差分写 ---------- */
static unsigned char *slot_buf;
static int slot_seq[NSLOT];
static _Atomic long job_next = 0;
static long n_jobs, rd_next = 0;
static volatile sig_atomic_t g_resize = 0;
static int g_gen = 0;

/* 块就绪 + 本槽上一圈已被消费 两个谓词, 每轮重算
 * 槽位闸门按圈数判定 (rd 越过本槽上一圈 occupant 才许渲染),
 * 旧写法 slot_seq[slot]<=rd 会把"过期标记"误当空闲, 允许跨圈插队踩掉主线程正等的标记 */
static int wait_worker(long job, int slot, long gen) {
    (void)slot;
    for (;;) {
        long s1 = cslot_of[job / CHUNKF];
        int ready = s1 && cpool[s1 - 1].state == 2;
        if (ready && (job < NSLOT || rd_next >= job - (NSLOT - 1))) return 1;
        if (g_stop || g_resize || gen != g_gen) return 0;
        pthread_cond_wait(&cv, &mu);
    }
}

static int wait_main(int slot) {
    for (;;) {
        if (slot_seq[slot] == rd_next + 1) return 1;
        if (g_stop || g_resize) return 0;
        pthread_cond_wait(&cv, &mu);
    }
}

/* 内联模式: 等目标块就绪 */
static int wait_chunk_locked(long c) {
    for (;;) {
        long s1 = cslot_of[c];
        if (s1 && cpool[s1 - 1].state == 2) return 1;
        if (g_stop) return 0;
        pthread_cond_wait(&cv, &mu);
    }
}

/* 主线程越过块边界时回收整块 */
static void release_passed_chunks_locked(void) {
    long nc = rd_next / CHUNKF;
    while (cur_chunk < nc) {
        long s1 = cslot_of[cur_chunk];
        if (s1 && cpool[s1 - 1].state == 2) {
            free(cpool[s1-1].fr); free(cpool[s1-1].ct); free(cpool[s1-1].fc);
            cpool[s1-1].fr = NULL; cpool[s1-1].ct = NULL; cpool[s1-1].fc = NULL;
            cpool[s1-1].state = 0;
            cslot_of[cur_chunk] = 0;
        }
        cur_chunk++;
        pthread_cond_broadcast(&cv);
    }
}

static void *worker(void *arg) {
    (void)arg;
    long gen = g_gen;
    for (;;) {
        if (gen != g_gen) break;
        long job = atomic_fetch_add(&job_next, 1);
        if (job >= n_jobs) break;
        int slot = (int)(job % NSLOT);
        unsigned char *tb = slot_buf + (size_t)slot * TICK_SZ;
        pthread_mutex_lock(&mu);
        int ok = wait_worker(job, slot, gen);
        pthread_mutex_unlock(&mu);
        if (!ok) break;
        render_frame(job, tb);
        pthread_mutex_lock(&mu);
        if (gen == g_gen) slot_seq[slot] = job + 1;
        pthread_cond_broadcast(&cv);
        pthread_mutex_unlock(&mu);
        if (gen != g_gen) break;
    }
    return NULL;
}

static void on_sigint(int s) { (void)s; g_stop = 1; }
static void on_sigwinch(int s) { (void)s; g_resize = 1; }

static int cols_opt = 0;
static void setup_canvas(void) {
    int cols = cols_opt;
    if (!cols) {
        struct winsize ws;
        int term = (ioctl(1, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 1) ? (int)ws.ws_col : 0;
        cols = 100;
        if (term > 1 && cols > term - 1) cols = term - 1;
    }
    W = (uint32_t)(2.0 * W0) < (uint32_t)cols ? 2 * W0 : (uint32_t)cols;
    H = (uint32_t)lround((double)W * H0 / (2.0 * W0));
    if (H < 8) H = 8;
    {
        struct winsize ws;
        int rows = (ioctl(1, TIOCGWINSZ, &ws) == 0 && ws.ws_row > 4) ? ws.ws_row : 30;
        if ((int)H > rows - 3) {
            H = rows - 3;
            uint32_t w2 = (uint32_t)lround(2.0 * H * W0 / (double)H0);
            if (w2 < W) W = w2;
        }
    }
    if (W < 8) W = 8;
    SX = (double)(W - 1) / (W0 - 1);
    SY = (double)(H - 1) / (H0 - 1);
    TICK_SZ = 3 + (size_t)H * (W + 1);
    emit_buf_reserve();
}

static void parse_blob(void) {
    g_blob = _binary_badapple_blob_bin_start;
    g_blob_len = (size_t)(_binary_badapple_blob_bin_end - _binary_badapple_blob_bin_start);
    if (g_blob_len < 32 || memcmp(g_blob, "BAP3", 4)) {
        fprintf(stderr, "badapple-c: blob is not BAP3\n");
        exit(1);
    }
    W0 = get_u32(g_blob + 4);  H0 = get_u32(g_blob + 8);
    FPS = get_u32(g_blob + 12); TOTAL = get_u32(g_blob + 16);
    NCHUNK = get_u32(g_blob + 20); CHUNKF = get_u32(g_blob + 24);
    g_chk = malloc(sizeof(ChunkEnt) * NCHUNK);
    const uint8_t *t = g_blob + 28;
    for (uint32_t i = 0; i < NCHUNK; i++) {
        g_chk[i].off = get_u32(t); g_chk[i].clen = get_u32(t + 4);
        g_chk[i].rlen = get_u32(t + 8); g_chk[i].first = get_u32(t + 12);
        t += 16;
        if ((size_t)g_chk[i].off + g_chk[i].clen > g_blob_len ||
            g_chk[i].first != i * CHUNKF) {
            fprintf(stderr, "badapple-c: corrupt chunk table\n");
            exit(1);
        }
    }
    cslot_of = calloc(NCHUNK, sizeof(long));
    if (!cslot_of) exit(1);
}

int main(int argc, char **argv) {
    int test_frame = -1, selftest = 0;
    double fps = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            fprintf(stderr,
                "badapple — 傅里叶级数终端画笔\n"
                "用法: ./badapple [选项]\n"
                "  --fps F       播放帧率, 默认 30\n"
                "  --threads N   预烤线程数, 默认自动 (画布 >= %d 列才开, 否则内联); 1=强制内联\n"
                "  --dthreads N  解压线程数, 默认 4\n"
                "  --cols C      画布列数, 默认 100 (超出终端宽自动收窄)\n"
                "  --test K      渲染离第 K 帧最近的选中帧后退出\n"
                "  --selftest    全块串行解压并输出校验和后退出\n"
                "  -h, --help    显示本帮助\n", WORKER_COLS);
            return 0;
        }
        else if (!strcmp(argv[i], "--fps") && i+1 < argc) fps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--threads") && i+1 < argc) NTHREAD = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--dthreads") && i+1 < argc) NDEC = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--cols") && i+1 < argc) cols_opt = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--char") && i+1 < argc) g_fg = argv[++i];
        else if (!strcmp(argv[i], "--test") && i+1 < argc) test_frame = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--selftest")) selftest = 1;
    }
    parse_blob();
    if (!fps) fps = 30;
    if (NTHREAD < 0) NTHREAD = 0;
    if (NTHREAD > 64) NTHREAD = 64;
    if (NDEC < 0) NDEC = 0;
    if (NDEC == 0) {
        long onl = sysconf(_SC_NPROCESSORS_ONLN);
        NDEC = onl > 4 ? 4 : (onl < 1 ? 1 : (int)onl);
    }
    tables_init();
    xz_crc32_init();

    setup_canvas();
    g_inline = (NTHREAD <= 1) || W < WORKER_COLS;
    if (!g_inline && NTHREAD == 0) NTHREAD = 4;

    /* --selftest: 全块串行解压, 输出逐值校验和 */
    if (selftest) {
        RefSlot *refs = calloc(NREF, sizeof(RefSlot));
        CSlot cs = {0};
        struct xz_dec *dec = xz_dec_init(XZ_SINGLE, 0);
        uint8_t *raw = NULL; size_t rawcap = 0;
        double sum = 0; uint32_t tf = 0, tc = 0;
        if (!refs || !dec) return 1;
        for (uint32_t c = 0; c < NCHUNK; c++) {
            if (rawcap < g_chk[c].rlen) {
                free(raw);
                rawcap = g_chk[c].rlen + g_chk[c].rlen / 4;
                raw = malloc(rawcap);
            }
            if (!raw || xz_chunk(&g_chk[c], raw, dec) != 0 ||
                undiff_chunk(raw, g_chk[c].rlen, &cs, refs) != 0) {
                fprintf(stderr, "selftest: chunk %u failed\n", c);
                return 1;
            }
            for (uint32_t f = 0; f < cs.nfr; f++)
                for (int k = 0; k < cs.fr[f].nct; k++) {
                    const Contour *ct = &cs.fr[f].ct[k];
                    int n = 2 * (ct->k1 - ct->k0 + 1);
                    for (int q = 0; q < n; q++) sum += ct->c[q];
                    tc++;
                }
            tf += cs.nfr;
            free_slot_arrays(&cs);
        }
        printf("selftest: %u frames, %u contours, coeff_sum=%.6f\n", tf, tc, sum);
        free(raw); free(refs); xz_dec_end(dec);
        return 0;
    }

    long n_sel = TOTAL;
    n_jobs = n_sel;
    fprintf(stderr, "badapple-c: %ux%u, %ld frames, %u chunks, render=%s decode=%d\n",
            W, H, n_sel, NCHUNK, g_inline ? "inline" : "worker", NDEC);

    /* --test: 同步解目标块, 渲一帧即走 */
    if (test_frame >= 0) {
        long bfi = 0, bd = TOTAL;
        for (long s = 0; s < n_sel; s++) {
            long fi = n_sel > 1 ? s * (TOTAL - 1) / (n_sel - 1) : 0;
            long d = fi > test_frame ? fi - test_frame : test_frame - fi;
            if (d < bd) { bd = d; bfi = fi; }
        }
        long c = bfi / CHUNKF;
        cslot_of[c] = 1;
        cpool[0].state = 2; cpool[0].chunk = c;
        struct xz_dec *dec = xz_dec_init(XZ_SINGLE, 0);
        uint8_t *raw = malloc(g_chk[c].rlen);
        RefSlot *refs = calloc(NREF, sizeof(RefSlot));
        if (!raw || !refs || !dec || xz_chunk(&g_chk[c], raw, dec) != 0 ||
            undiff_chunk(raw, g_chk[c].rlen, &cpool[0], refs) != 0) {
            fprintf(stderr, "badapple-c: test chunk decode failed\n");
            return 1;
        }
        free(raw); free(refs); xz_dec_end(dec);
        unsigned char *tb = malloc(TICK_SZ);
        render_frame(bfi, tb);
        fwrite(tb + 3, 1, TICK_SZ - 3, stdout);
        return 0;
    }

    signal(SIGINT, on_sigint);
    signal(SIGWINCH, on_sigwinch);
    pthread_t th[64], dth[16];
    if (NDEC > 16) NDEC = 16;
    for (int i = 0; i < NDEC; i++) pthread_create(&dth[i], NULL, decoder, NULL);

    int fd = 1;
    full_write(fd, "\x1b[?25l\x1b[2J", 10);
    unsigned char *tick = NULL;

    if (!g_inline) {
        slot_buf = malloc((size_t)NSLOT * TICK_SZ);
        for (long s = 0; s < NSLOT; s++)
            blank_tick(slot_buf + (size_t)s * TICK_SZ);
        for (int i = 0; i < NTHREAD; i++) pthread_create(&th[i], NULL, worker, NULL);
    } else {
        tick = malloc(TICK_SZ);
        blank_tick(tick);
    }

    struct timespec ts;
    double beat = 1.0 / fps, next = 0;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    next = ts.tv_sec + ts.tv_nsec * 1e-9;
    while (!g_stop && rd_next < n_jobs) {
        if (g_resize) {
            g_resize = 0;
            usleep(150000);
            if (g_resize) continue;
            if (g_inline) {
                /* 内联: 帧是现渲的, 只需换画布重画; 解码槽与几何无关 */
                free(g_prev); g_prev = NULL;
                setup_canvas();
                free(tick);
                tick = malloc(TICK_SZ);
                blank_tick(tick);
                full_write(fd, "\x1b[2J", 4);
            } else {
                pthread_mutex_lock(&mu);
                g_gen++;
                rd_next = atomic_load(&job_next);
                memset(slot_seq, 0, sizeof slot_seq);
                release_passed_chunks_locked();
                pthread_cond_broadcast(&cv);
                pthread_mutex_unlock(&mu);
                for (int i = 0; i < NTHREAD; i++) pthread_join(th[i], NULL);
                free(g_prev); g_prev = NULL;
                setup_canvas();
                free(slot_buf);
                slot_buf = malloc((size_t)NSLOT * TICK_SZ);
                for (long s = 0; s < NSLOT; s++)
                    blank_tick(slot_buf + (size_t)s * TICK_SZ);
                full_write(fd, "\x1b[2J", 4);
                for (int i = 0; i < NTHREAD; i++) pthread_create(&th[i], NULL, worker, NULL);
            }
            fprintf(stderr, "badapple-c: resize -> %ux%u\n", W, H);
            if (rd_next >= n_jobs) break;
            next = -1;
        }
        if (g_inline) {
            long c = rd_next / CHUNKF;
            pthread_mutex_lock(&mu);
            release_passed_chunks_locked();
            int ok = wait_chunk_locked(c);
            pthread_cond_broadcast(&cv);
            pthread_mutex_unlock(&mu);
            if (!ok) break;
            render_frame(rd_next, tick);
            emit_frame(fd, tick + 3);
            rd_next++;
        } else {
            int slot = (int)(rd_next % NSLOT);
            pthread_mutex_lock(&mu);
            int ok = wait_main(slot);
            pthread_mutex_unlock(&mu);
            if (!ok) continue;
            unsigned char *tb = slot_buf + (size_t)slot * TICK_SZ;
            emit_frame(fd, tb + 3);
            rd_next++;
            pthread_mutex_lock(&mu);
            release_passed_chunks_locked();
            pthread_cond_broadcast(&cv);
            pthread_mutex_unlock(&mu);
        }
        next += beat;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double now = ts.tv_sec + ts.tv_nsec * 1e-9;
        if (next - now > 0) {
            ts.tv_sec = (time_t)next;
            ts.tv_nsec = (long)((next - ts.tv_sec) * 1e9);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        } else {
            next = now;
        }
    }
    full_write(fd, "\x1b[?25h\n", 7);
    if (!g_inline)
        for (int i = 0; i < NTHREAD; i++) pthread_join(th[i], NULL);
    g_stop = 1;
    pthread_mutex_lock(&mu);
    pthread_cond_broadcast(&cv);
    pthread_mutex_unlock(&mu);
    for (int i = 0; i < NDEC; i++) pthread_join(dth[i], NULL);
    return 0;
}
