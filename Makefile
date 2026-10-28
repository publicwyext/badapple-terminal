CC      ?= gcc
CFLAGS  ?= -O3 -march=native -flto -fno-plt -fomit-frame-pointer \
           -fno-math-errno -fno-asynchronous-unwind-tables -fno-unwind-tables \
           -fcf-protection=none -fno-stack-protector -fno-ident
CFLAGS  += -ffunction-sections -fdata-sections
LDFLAGS += -Wl,--gc-sections,--build-id=none -Wl,-z,noseparate-code,-z,norelro
LDLIBS  += -lm -lpthread -ldl

VIDEO   ?=                 # make gen VIDEO=my.mp4

XZ_SRC := $(wildcard xz_embedded/*.c)

ifeq ($(NO_AUDIO),1)
CFLAGS += -DNO_AUDIO
endif

all: badapple

# bring-your-own-video: video -> contour JSON -> BAP3 blob (+MP3 audio tail) -> blob.o
ifneq ($(VIDEO),)
ba_aud.mp3: $(VIDEO)
	ffmpeg -y -v error -i $< -vn -ac 1 -ar 16000 -b:a 16k $@
blob.o: badapple.json ba_aud.mp3
	python3 make_blob.py $< badapple_blob.bin --audio ba_aud.mp3
	ld -r -b binary -o $@ badapple_blob.bin

badapple.json: $(VIDEO)
	python3 encode_planb.py $(VIDEO) $@
else
blob.o: badapple_blob.bin
	@test -f $< || { echo "no badapple_blob.bin — bring your own video first:"; echo "  make gen VIDEO=my.mp4   (needs python3-numpy + ffmpeg)"; exit 1; }
	ld -r -b binary -o $@ $<
endif

audio_impl.o: audio_impl.c miniaudio.h audio.h
	$(CC) $(subst -O3,-Os,$(CFLAGS)) -c audio_impl.c -o $@

badapple: badapple.c audio.h $(XZ_SRC) blob.o audio_impl.o
	$(CC) $(CFLAGS) badapple.c audio_impl.c $(XZ_SRC) blob.o -o $@ $(LDFLAGS) $(LDLIBS)
	strip --strip-all -R .comment -R .note -R .note.gnu.build-id $@

clean:
	rm -f badapple blob.o badapple_blob.bin badapple.json ba_aud.mp3 audio_impl.o

# ---- cross builds: badapple-x86_64 / badapple-aarch64 (make cross) ----
# x86_64 needs -march=x86-64-v2 (cross gcc has no -march=native); aarch64 keeps it
CFLAGS_X86     := $(filter-out -march=native,$(CFLAGS)) -march=x86-64-v2
CFLAGS_AARCH64 := $(CFLAGS)

define CROSS_RULE
badapple-$(1): badapple.c audio.h $$(XZ_SRC) blob-$(1).o audio-$(1).o
	$$(TRIPLE_$(1))-gcc $$(CFLAGS_$(2)) badapple.c audio_impl.c $$(XZ_SRC) blob-$(1).o -o $$@ $$(LDFLAGS) $$(LDLIBS)
	$$(TRIPLE_$(1))-strip --strip-all -R .comment -R .note -R .note.gnu.build-id $$@

blob-$(1).o: badapple_blob.bin
	$$(TRIPLE_$(1))-ld -r -b binary -o $$@ $$<

audio-$(1).o: audio_impl.c miniaudio.h audio.h
	$$(TRIPLE_$(1))-gcc $$(subst -O3,-Os,$$(CFLAGS_$(2))) -c audio_impl.c -o $$@
endef

TRIPLE_x86_64  := x86_64-linux-gnu
TRIPLE_aarch64 := aarch64-linux-gnu
$(eval $(call CROSS_RULE,x86_64,X86))
$(eval $(call CROSS_RULE,aarch64,AARCH64))

cross: badapple-x86_64 badapple-aarch64
	@readelf -h badapple-x86_64  | grep -q 'Machine:.*X86-64'  || { echo "FATAL: x86_64 build is not x86-64"; file badapple-x86_64; exit 1; }
	@readelf -h badapple-aarch64 | grep -q 'Machine:.*AArch64' || { echo "FATAL: aarch64 build is not aarch64"; file badapple-aarch64; exit 1; }
	@file badapple-x86_64 badapple-aarch64 | sed 's/, BuildID[^,]*//'

.PHONY: all clean gen cross win clean-win
gen: all

# ---- Windows cross build: badapple.exe (make win; needs gcc-mingw-w64-x86-64) ----
# Targets Windows 10 1809+ (ANSI/VT console). Fully static — no MinGW runtime
# DLLs. Clocks/pthreads come from winpthreads; audio is miniaudio WASAPI.
TRIPLE_WIN := x86_64-w64-mingw32
CFLAGS_WIN := -O3 -ffunction-sections -fdata-sections \
              -fno-asynchronous-unwind-tables -fno-unwind-tables \
              -fno-stack-protector -fno-ident -D_WIN32_WINNT=0x0A00
LDLIBS_WIN := -static -lwinpthread -lole32 -lwinmm -lm

win: badapple.exe
	@file badapple.exe | grep -q 'PE32+.*x86-64' || { echo "FATAL: badapple.exe is not Windows x86-64"; file badapple.exe; exit 1; }
	@cp badapple.exe badapple-windows-x86_64.exe
	@file badapple-windows-x86_64.exe | sed 's/, stripped[^,]*//'

blob-win.o: badapple_blob.bin
	$(TRIPLE_WIN)-ld -r -b binary -o $@ $<

badapple.exe: badapple.c win_compat.h audio.h $(XZ_SRC) blob-win.o audio_impl.c miniaudio.h
	$(TRIPLE_WIN)-gcc $(CFLAGS_WIN) badapple.c audio_impl.c $(XZ_SRC) blob-win.o -o $@ -Wl,--gc-sections $(LDLIBS_WIN)
	$(TRIPLE_WIN)-strip --strip-all $@

clean-win:
	rm -f badapple.exe blob-win.o
