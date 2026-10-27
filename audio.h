#ifndef AUD_H
#define AUD_H
#include <stddef.h>
#ifndef NO_AUDIO
int aud_start(const void *p, size_t n);
void aud_stop(void);
#else
static inline int aud_start(const void *p, size_t n) { (void)p; (void)n; return 0; }
static inline void aud_stop(void) { }
#endif
#endif
