/* win_compat.h — Windows shim for badapple.c (_WIN32 only; include instead of
 * unistd/fcntl/sys/ioctl.h)
 *
 * Provides:
 *   - ANSI/VT processing enable on stdout console (Windows 10 1809+)
 *   - console window size (win_term_size)
 *   - resize polling — Windows has no SIGWINCH (win_poll_resize)
 *
 * Clocks & threads need no shims: mingw-w64 winpthreads (-lwinpthread)
 * exports clock_gettime / clock_nanosleep / pthread_* with POSIX semantics.
 */
#ifndef BA_WIN_COMPAT_H
#define BA_WIN_COMPAT_H

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <io.h>
#include <sys/types.h>

/* libc names mingw doesn't export */
#define write(fd, buf, n) _write(fd, buf, n)
#define usleep(us)        Sleep((DWORD)(((us) + 999) / 1000))

#define _SC_NPROCESSORS_ONLN 0
static long sysconf(int name) {
    (void)name;
    SYSTEM_INFO si;
    GetSystemInfo(&si);
    return (long)si.dwNumberOfProcessors;
}

static void win_enable_vt(void) {
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    if (h != INVALID_HANDLE_VALUE && GetConsoleMode(h, &mode))
        SetConsoleMode(h, mode | ENABLE_VIRTUAL_TERMINAL_PROCESSING |
                              ENABLE_PROCESSED_OUTPUT);
}

#ifdef __GNUC__
__attribute__((constructor))
#endif
static void win_vt_ctor(void) { win_enable_vt(); }

/* visible window size (not the scrollback buffer) */
static int win_term_size(int *cols, int *rows) {
    CONSOLE_SCREEN_BUFFER_INFO csbi;
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    if (h == INVALID_HANDLE_VALUE || !GetConsoleScreenBufferInfo(h, &csbi))
        return -1;
    if (cols) *cols = (int)(csbi.srWindow.Right - csbi.srWindow.Left + 1);
    if (rows) *rows = (int)(csbi.srWindow.Bottom - csbi.srWindow.Top + 1);
    return 0;
}

/* 1 when the console window changed size since last call */
static int win_poll_resize(void) {
    static int last_c = -1, last_r = -1;
    int c = -1, r = -1;
    if (win_term_size(&c, &r) != 0) return 0;
    if (last_c < 0) { last_c = c; last_r = r; return 0; }
    if (c != last_c || r != last_r) { last_c = c; last_r = r; return 1; }
    return 0;
}

#endif /* BA_WIN_COMPAT_H */
