/* badapple.c — 傅里叶级数画笔, 终端播放 (C 重写·极致性能版)
 * 数据嵌入: badapple_blob.bin 由 ld -r -b binary 连入, 无外部依赖
 * 渲染: 只用 '#' (ASCII), 画布每格 1 字节
 * 并行: pthread 流水线 — N 个工人按序预烤帧, 主线程只差分写 + 节拍休眠
 * 差分: 帧间只输出变化的格子 (擦旧加新), 不再全屏重画; SIGWINCH 自动适配窗口
 * 用法: ./badapple_c badapple.json  <-- 参数忽略, 数据已在体内
 *   可选: --fps F  --threads N  --cols C  --test K (渲染第K帧后退出)
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <time.h>
#include <signal.h>
#include <sys/ioctl.h>

extern const unsigned char _binary_badapple_blob_bin_start[];
extern const unsigned char _binary_badapple_blob_bin_end[];

#define SAMPLES 256
#define NSLOT 64
#define BLK '#'

static uint32_t W0, H0, FPS, TOTAL;         /* blob 元数据 */
static uint32_t W, H;                        /* 画布尺寸 */
static double SX, SY;
static int NTHREAD = 4;
static size_t TICK_SZ;                       /* 单动画帧字节数 */
static volatile sig_atomic_t g_stop = 0;

typedef struct { int32_t k0, k1; uint32_t n; const float *c; } Contour;
static Contour *g_ct;                        /* 全部轮廓 */
static uint32_t *g_coff;                     /* 每帧轮廓起点 [TOTAL+1] */

typedef struct { Contour *ct; int nct; } Frame;
static Frame *g_fr;

/* ---------- 级数合成: 直角坐标直接累加 ---------- */
static void decode(const Contour *ct, double *pts) {
    int K = ct->k1 - ct->k0 + 1;
    for (int s = 0; s < (int)ct->n; s++) {
        double t = (double)s / ct->n, ph;
        double zr = ct->c[0], zi = ct->c[1];
        for (int j = 1; j < K; j++) {
            ph = 2.0 * M_PI * (ct->k0 + j) * t;
            double cr = ct->c[2*j], ci = ct->c[2*j+1];
            zr += cr * cos(ph) - ci * sin(ph);
            zi += cr * sin(ph) + ci * cos(ph);
        }
        pts[2*s] = zr; pts[2*s+1] = zi;
    }
}

/* ---------- 光栅化 ---------- */
static inline void plot(unsigned char *buf, int x, int y) {
    if (x < 0 || x >= (int)W || y < 0 || y >= (int)H) return;
    buf[(size_t)y * (W + 1) + x] = BLK;
}

/* 整条链画到 buf (闭合: 首尾相接) */
static void plot_all(const double *pts, int npts, unsigned char *buf) {
    int m = npts;
    for (int s = 0; s < m; s++) {
        double x0 = pts[2*s] * SX,        y0 = pts[2*s+1] * SY;
        double x1 = pts[2*((s+1)%m)] * SX, y1 = pts[2*((s+1)%m)+1] * SY;
        int steps = (int)(fmax(fabs(x1-x0), fabs(y1-y0))) + 2;
        for (int i = 0; i < steps; i++) {
            double t = (double)i / steps;
            int px = (int)lround(x0 + (x1-x0)*t), py = (int)lround(y0 + (y1-y0)*t);
            plot(buf, px, py);
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

/* ---------- 一帧 -> 完整画布 (差分引擎取最终图像, 不再分拍) ---------- */
static void render_frame(int fi, unsigned char *t) {
    blank_tick(t);
    Frame *fr = &g_fr[fi];
    if (fr->nct <= 0) return;
    double tmp[2*SAMPLES];
    for (uint32_t k = 0; k < fr->nct; k++) {
        decode(&fr->ct[k], tmp);
        plot_all(tmp, SAMPLES, t);
    }
}

/* ---------- 差分输出: 只写与上一帧不同的格子 ---------- */
static unsigned char *g_prev = NULL;         /* [H][W+1] 上一帧画布 */

static void emit_frame(int fd, const unsigned char *cur) {
    char pre[20];
    if (!g_prev) {                                /* 首帧/换尺寸: 整行快路径 */
        g_prev = malloc((size_t)H * (W + 1));
        memset(g_prev, ' ', (size_t)H * (W + 1));
        write(fd, "\x1b[2J", 4);
        for (uint32_t r = 0; r < H; r++) {
            int n = snprintf(pre, sizeof pre, "\x1b[%u;1H", r + 1);
            write(fd, pre, n);
            write(fd, cur + (size_t)r * (W + 1), W);
        }
        write(fd, "\x1b[H", 3);
        memcpy(g_prev, cur, (size_t)H * (W + 1));
        return;
    }
    for (uint32_t r = 0; r < H; r++) {
        const unsigned char *a = g_prev + (size_t)r * (W + 1);
        const unsigned char *b = cur   + (size_t)r * (W + 1);
        uint32_t c = 0;
        while (c < W) {
            while (c < W && a[c] == b[c]) c++;
            if (c >= W) break;
            uint32_t c0 = c;
            while (c < W && a[c] != b[c]) c++;
            int n = snprintf(pre, sizeof pre, "\x1b[%u;%uH", r + 1, c0 + 1);
            write(fd, pre, n);
            write(fd, b + c0, c - c0);
        }
    }
    write(fd, "\x1b[H", 3);                  /* 光标归位 */
    memcpy(g_prev, cur, (size_t)H * (W + 1));
}

/* ---------- 流水线: 工人按序预烤, 主线程差分写 ---------- */
static unsigned char *slot_buf;                      /* [NSLOT][TICK_SZ] */
static int slot_seq[NSLOT];
static _Atomic long job_next = 0;
static long n_jobs, rd_next = 0;
static volatile sig_atomic_t g_resize = 0;
static int g_gen = 0;                                /* 重建代数, 工人据此换班 */
static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cv = PTHREAD_COND_INITIALIZER;

/* 带超时的条件等待: 每轮重算谓词, broadcast 丢失也有轮询兜底; 信号/换班可打断 */
static int wait_worker(int slot, long gen) {
    struct timespec ts;
    for (;;) {
        if (slot_seq[slot] <= rd_next) return 1;
        if (g_stop || g_resize || gen != g_gen) return 0;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 50000000;                      /* 50ms 轮询兜底 */
        if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
        pthread_cond_timedwait(&cv, &mu, &ts);
    }
}

static int wait_main(int slot) {
    struct timespec ts;
    for (;;) {
        if (slot_seq[slot] == rd_next + 1) return 1;
        if (g_stop || g_resize) return 0;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 50000000;
        if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
        pthread_cond_timedwait(&cv, &mu, &ts);
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
        int ok = wait_worker(slot, gen);
        pthread_mutex_unlock(&mu);
        if (!ok) break;
        render_frame((int)job, tb);
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

/* ---------- 画布尺寸: 默认宽100, 最多不超终端; --cols 可指定 ---------- */
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
}

int main(int argc, char **argv) {
    int test_frame = -1;
    double fps = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")) {
            fprintf(stderr,
                "badapple_c — 傅里叶级数终端画笔\n"
                "用法: ./badapple_c [选项]\n"
                "  --fps F      播放帧率, 默认 30\n"
                "  --threads N  渲染线程数, 默认 4\n"
                "  --cols C     画布列数, 默认 100 (超出终端宽自动收窄)\n"
                "  --test K     渲染离第 K 帧最近的选中帧后退出\n"
                "  -h, --help   显示本帮助\n");
            return 0;
        }
        else if (!strcmp(argv[i], "--fps") && i+1 < argc) fps = atof(argv[++i]);
        else if (!strcmp(argv[i], "--threads") && i+1 < argc) NTHREAD = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--cols") && i+1 < argc) cols_opt = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--test") && i+1 < argc) test_frame = atoi(argv[++i]);
    }
    const uint32_t *hdr = (const uint32_t *)_binary_badapple_blob_bin_start;
    W0 = hdr[0]; H0 = hdr[1]; FPS = hdr[2]; TOTAL = hdr[3];
    if (!fps) fps = 30;
    if (NTHREAD < 1) NTHREAD = 1;
    if (NTHREAD > 64) NTHREAD = 64;

    /* 帧索引: 一次走查 */
    g_ct = malloc(sizeof(Contour) * (size_t)TOTAL * 8);
    g_coff = malloc(sizeof(uint32_t) * (size_t)(TOTAL + 1));
    g_fr = calloc(TOTAL, sizeof(Frame));
    const uint32_t *p = hdr + 4;
    uint32_t nct_all = 0;
    for (uint32_t i = 0; i < TOTAL; i++) {
        uint32_t cnt = *p++;
        g_coff[i] = nct_all;
        for (uint32_t k = 0; k < cnt; k++) {
            int32_t k0 = (int32_t)p[0], k1 = (int32_t)p[1];
            uint32_t n = p[2];
            g_ct[nct_all].k0 = k0; g_ct[nct_all].k1 = k1;
            g_ct[nct_all].n = n; g_ct[nct_all].c = (const float *)(p + 3);
            nct_all++;
            p += 3 + 2u * (uint32_t)(k1 - k0 + 1);
        }
    }
    g_coff[TOTAL] = nct_all;
    for (uint32_t i = 0; i < TOTAL; i++) {
        g_fr[i].ct = &g_ct[g_coff[i]];
        g_fr[i].nct = (int)(g_coff[i+1] - g_coff[i]);
    }

    setup_canvas();

    long n_sel = TOTAL;                           /* 逐帧, 无采样 */
    n_jobs = n_sel;
    fprintf(stderr, "badapple-c: %ux%u, %ld frames, threads=%d\n",
            W, H, n_sel, NTHREAD);

    slot_buf = malloc((size_t)NSLOT * TICK_SZ);
    for (long s = 0; s < NSLOT; s++)
        blank_tick(slot_buf + (size_t)s * TICK_SZ);

    /* --test: 渲染离第K帧最近的选中帧, 只出最后一笔 */
    if (test_frame >= 0) {
        long bfi = 0, bd = 1L << 60;
        for (long s = 0; s < n_sel; s++) {
            long fi = n_sel > 1 ? s * (TOTAL - 1) / (n_sel - 1) : 0;
            long d = fi > test_frame ? fi - test_frame : test_frame - fi;
            if (d < bd) { bd = d; bfi = fi; }
        }
        unsigned char *tb = malloc(TICK_SZ);
        render_frame((int)bfi, tb);
        fwrite(tb + 3, 1, TICK_SZ - 3, stdout);
        free(tb);
        return 0;
    }

    signal(SIGINT, on_sigint);
    signal(SIGWINCH, on_sigwinch);
    pthread_t th[64];
    for (int i = 0; i < NTHREAD; i++) pthread_create(&th[i], NULL, worker, NULL);

    int fd = 1;
    write(fd, "\x1b[?25l\x1b[2J", 7);
    struct timespec ts;
    double beat = 1.0 / fps, next = 0;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    next = ts.tv_sec + ts.tv_nsec * 1e-9;
    long written = 0;
    while (!g_stop && rd_next < n_jobs) {
        if (g_resize) {                           /* 窗口变了: 去抖后排水重建 */
            g_resize = 0;
            usleep(150000);                       /* 吸收拖动中的 WINCH 风暴 */
            if (g_resize) continue;               /* 还在拖, 等下一轮再重建 */
            pthread_mutex_lock(&mu);
            g_gen++;                              /* 锁内换班, 消除可见性竞态 */
            rd_next = atomic_load(&job_next);     /* 丢弃未读的旧尺寸帧 */
            memset(slot_seq, 0, sizeof slot_seq);
            pthread_cond_broadcast(&cv);          /* 唤醒工人早退 */
            pthread_mutex_unlock(&mu);
            for (int i = 0; i < NTHREAD; i++) pthread_join(th[i], NULL);
            free(g_prev); g_prev = NULL;          /* 强制下一帧全量重出 */
            setup_canvas();
            free(slot_buf);
            slot_buf = malloc((size_t)NSLOT * TICK_SZ);
            for (long s = 0; s < NSLOT; s++)
                blank_tick(slot_buf + (size_t)s * TICK_SZ);
            write(fd, "\x1b[2J", 4);
            fprintf(stderr, "badapple-c: resize -> %ux%u\n", W, H);
            for (int i = 0; i < NTHREAD; i++) pthread_create(&th[i], NULL, worker, NULL);
            if (rd_next >= n_jobs) break;
            next = -1;                            /* 重建耗时计入节拍, 防连发 */
        }
        int slot = (int)(rd_next % NSLOT);
        pthread_mutex_lock(&mu);
        int ok = wait_main(slot);
        pthread_mutex_unlock(&mu);
        if (!ok) continue;                        /* resize/stop: 回循环顶处理 */
        unsigned char *tb = slot_buf + (size_t)slot * TICK_SZ;
        emit_frame(fd, tb + 3);
        written++;
        next += beat;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        double now = ts.tv_sec + ts.tv_nsec * 1e-9;
        if (next - now > 0) {
            ts.tv_sec = (time_t)next;
            ts.tv_nsec = (long)((next - ts.tv_sec) * 1e9);
            clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
        } else if (next < 0) {
            next = now;                           /* 重建后重新起拍 */
        } else {
            next = now;                           /* 落后不追帧, 防雪崩 */
        }
        rd_next++;
        pthread_mutex_lock(&mu);
        pthread_cond_broadcast(&cv);              /* 通知工人槽位已空 */
        pthread_mutex_unlock(&mu);
    }
    write(fd, "\x1b[?25h\n", 7);
    for (int i = 0; i < NTHREAD; i++) pthread_join(th[i], NULL);
    return 0;
}
