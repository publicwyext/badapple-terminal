CC      ?= gcc
CFLAGS  ?= -O3 -march=native -flto -fno-plt -fomit-frame-pointer
CFLAGS  += -ffunction-sections -fdata-sections
LDFLAGS += -Wl,--gc-sections,--build-id=none
LDLIBS  += -lm -lpthread

VIDEO   ?=                 # make gen VIDEO=my.mp4

XZ_SRC := $(wildcard xz_embedded/*.c)

all: badapple

# bring-your-own-video: video -> contour JSON -> BAP2 blob -> blob.o
ifneq ($(VIDEO),)
blob.o: badapple.json
	python3 make_blob.py $< badapple_blob.bin
	ld -r -b binary -o $@ badapple_blob.bin

badapple.json: $(VIDEO)
	python3 encode_planb.py $(VIDEO) $@
else
blob.o: badapple_blob.bin
	@test -f $< || { echo "no badapple_blob.bin — bring your own video first:"; echo "  make gen VIDEO=my.mp4   (needs python3-numpy + ffmpeg)"; exit 1; }
	ld -r -b binary -o $@ $<
endif

badapple: badapple.c $(XZ_SRC) blob.o
	$(CC) $(CFLAGS) badapple.c $(XZ_SRC) blob.o -o $@ $(LDFLAGS) $(LDLIBS)
	strip --strip-all -R .comment -R .note -R .note.gnu.build-id $@

clean:
	rm -f badapple blob.o badapple_blob.bin badapple.json

.PHONY: all clean gen
gen: all
