#TC=~/zephyr-sdk-0.16.1/xtensa-intel_tgl_adsp_zephyr-elf/bin/xtensa-intel_tgl_adsp_zephyr-elf-
TC=/xtensa/XtDevTools/install/tools/RI-2021.6-linux/XtensaTools/bin/xt-
XTENSA_CORE=hifi4_8195_PROD

#CC=$(TC)gcc
CC=$(TC)clang

AS=$(TC)as
LD=$(TC)ld
OBJDUMP=$(TC)objdump
OBJCOPY=$(TC)objcopy

OBJS=test.o ctest.o timer.o

CFLAGS=-mabi=call0 -Os -Wall -std=c99 -I$(HOME)/z/modules/hal/xtensa/include

.c.o:
	$(CC) $(CFLAGS) -c -o $@ $<

.s.o:
	$(AS) -o $@ $<

img.bin: $(OBJS) test.ld
	$(LD) -o img.elf -T test.ld -Map=img.map $(OBJS)
	$(OBJDUMP) -j .text -D -S img.elf > img.lst
	$(OBJCOPY) -j .text -O binary img.elf $@

clean:
	rm -f *.bin *.elf *.o *.lst *.map
