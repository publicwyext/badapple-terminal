/* miniaudio 单头实现 + 内嵌 MP3 播放, 回调读断流即静音 */
#ifndef NO_AUDIO
#define MINIAUDIO_IMPLEMENTATION
#include "miniaudio.h"
#include "audio.h"
static ma_dr_mp3 l3;
static ma_device dev;
static int on;
static void cb(ma_device *d, void *o, const void *i, ma_uint32 n) {
    (void)i;
    if (ma_dr_mp3_read_pcm_frames_f32(&l3, n, (float *)o) < n)
        memset(o, 0, (size_t)n * d->playback.channels * sizeof(float));
}
int aud_start(const void *p, size_t n) {
    if (!p || !n || ma_dr_mp3_init_memory(&l3, p, n, NULL) != MA_TRUE) return 0;
    ma_device_config c = ma_device_config_init(ma_device_type_playback);
    c.playback.channels = l3.mp3FrameChannels;
    c.sampleRate = l3.mp3FrameSampleRate;
    c.dataCallback = cb;
    if (ma_device_init(NULL, &c, &dev) != MA_SUCCESS) { ma_dr_mp3_uninit(&l3); return 0; }
    if (ma_device_start(&dev) != MA_SUCCESS) { ma_device_uninit(&dev); ma_dr_mp3_uninit(&l3); return 0; }
    return on = 1;
}
void aud_stop(void) {
    if (!on) return;
    on = 0;
    ma_device_stop(&dev);
    ma_device_uninit(&dev);
    ma_dr_mp3_uninit(&l3);
}
#endif
