
# Zephyr SDK (has a weird bug with decimal printing and mtprintf...)
#TC=~/zephyr-sdk-0.16.1/xtensa-intel_tgl_adsp_zephyr-elf/bin/xtensa-intel_tgl_adsp_zephyr-elf-
#CC=$(TC)gcc

# Cadence xt-clang
TC=/xtensa/XtDevTools/install/tools/RI-2021.6-linux/XtensaTools/bin/xt-
CC=$(TC)clang

export XTENSA_CORE=hifi4_8195_PROD

DUT=dojo

AS=$(TC)as
LD=$(TC)ld
OBJDUMP=$(TC)objdump
OBJCOPY=$(TC)objcopy

OBJS=test.o ctest.o timer.o clk.o window_vectors.o

CFLAGS=-Os -Wall -ffunction-sections -fdata-sections -std=c99 -I$(HOME)/z/modules/hal/xtensa/include

# Useful for testing vs. window exception goofs
#CFLAGS += -mabi=call0

CFLAGS += -D_DATE=\"`date +%H:%M:%S`\"

.c.o:
	$(CC) $(CFLAGS) -c -o $@ $<

.s.o:
	$(AS) -o $@ $<

img.bin: $(OBJS) linker.x
	$(LD) -o img.elf -T linker.x -Map=img.map $(OBJS)
	$(OBJDUMP) -D -S img.elf > img.lst
	$(OBJCOPY) -j .text -j .z_xtensa_vectors -O binary img.elf $@

run: img.bin
	scp mt8195-load.py img.bin $(DUT):.
	ssh $(DUT) './mt8195-load.py stop; ./mt8195-load.py load sram img.bin; ./mt8195-load.py start; sleep 0.3; ./mt8195-load.py log'

linker.x: test.ld
	cpp -P > $@ < $<

test.ld: vecs.ld

vecs.ld: core-isa-dM.h
	./vecgen.py $< > $@

core-isa-dM.h: $(XTENSA_CORE).h
	gcc -E -dM -o $@ $<

clean:
	rm -f *.bin *.elf *.o *.lst *.map core-isa-dM.h vecs.ld linker.x
