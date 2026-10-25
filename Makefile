CC      ?= gcc
CFLAGS  ?= -O3 -march=native -flto -fno-plt -fomit-frame-pointer
CFLAGS  += -ffunction-sections -fdata-sections
LDFLAGS += -Wl,--gc-sections,--build-id=none
LDLIBS  += -lm -lpthread

XZ_SRC := $(wildcard xz_embedded/*.c)

all: badapple

blob.o: badapple_blob.bin
	ld -r -b binary -o $@ $<

badapple: badapple.c $(XZ_SRC) blob.o
	$(CC) $(CFLAGS) badapple.c $(XZ_SRC) blob.o -o $@ $(LDFLAGS) $(LDLIBS)
	strip --strip-all -R .comment -R .note -R .note.gnu.build-id $@

clean:
	rm -f badapple blob.o

.PHONY: all clean
