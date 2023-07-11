TC=~/zephyr-sdk-0.16.1/xtensa-intel_tgl_adsp_zephyr-elf/bin/xtensa-intel_tgl_adsp_zephyr-elf

AS=$(TC)-as
LD=$(TC)-ld
OBJDUMP=$(TC)-objdump
OBJCOPY=$(TC)-objcopy

img.bin: test.s test.ld
	$(AS) -o test.o test.s
	$(LD) -o img.elf -T test.ld test.o
	$(OBJDUMP) -j .text -D img.elf
	$(OBJCOPY) -j .text -O binary img.elf $@

clean:
	rm -f img.bin img.elf test.o
