#include <string.h>

extern char _mtk_adsp_sram_end[];
#define SRAM_START 0x40000000
#define SRAM_END   0x40040000

extern char _mtk_adsp_dram_end[];
#define DRAM_START 0x60000000
#define DRAM_END   0x61100000

/* This is the true boot vector.  This device allows for direct
 * setting of the alternate reset vector, so we let it link wherever
 * it lands and extract its address in the loader.  This represents
 * the minimum amount of effort required to successfully call a C
 * function (and duplicates a few versions elsewhere in the tree:
 * really this should move to the arch layer).
 */
__asm__(".align 4\n\t"
        ".global mtk_adsp_boot_entry\n\t"
        "mtk_adsp_boot_entry:\n\t"
        "  movi  a0, 0x4002f\n\t" /* WOE|EXCM|INTLVL=15 */
        "  wsr   a0, PS\n\t"
        "  movi  a0, 0\n\t"
        "  wsr   a0, WINDOWBASE\n\t"
        "  movi  a0, 1\n\t"
        "  wsr   a0, WINDOWSTART\n\t"
        "  rsync\n\t"
        "  movi  a1, 0x40040000\n\t"
        "  call4 c_boot\n\t");

/* Initial MPU configuration, needed to enable caching */
static void enable_mpu(void)
{
	/* FIXME: should get regions from devicetree */
	/* FIXME: early DRAM (linked image) should be cached too! */
	static const unsigned int mpu[][2] = {
		{ 0x00000000, 0x06000 }, /* inaccessible null region */
		{ 0x10000000, 0x06f00 }, /* MMIO registers */
		{ 0x1d000000, 0x06000 }, /* inaccessible */
		{ SRAM_START, 0xf7f00 }, /* 256k cached SRAM */
		{ SRAM_END,   0x06000 }, /* inaccessible */
		{ DRAM_START, 0x06f00 }, /* 17M uncached DRAM */
		{ DRAM_END,   0x06000 }, /* inaccessible */
	};

	/* Must write BACKWARDS FROM THE END to avoid introducing a
	 * non-monotonic segment at the current instruction fetch.  The
	 * exception triggers even if all the segments involved are
	 * disabled!
	 */
	int nseg = sizeof(mpu)/sizeof(mpu[0]);
	for (int i = 31; i >= 32 - nseg; i--) {
		int mpuidx = i - (32 - nseg);
		unsigned int addren = mpu[mpuidx][0] | 1;
		unsigned int segprot = (mpu[mpuidx][1]) | i;

		/* If an existing instruction fetch is in the same segment,
		 * wptlb must be preceded by a memw in the same cache line.
		 */
		__asm__ volatile("j 1f\n"
				 ".align 8\n"
				 "1:\n"
				 "memw\n"
#ifdef __XCC__
				 "wptlb %1, %0" /* same ... */
#else
				 "wdtlb %1, %0" /* ... insn format */
#endif
				 :: "r"(addren), "r"(segprot));
	}
}

void c_boot(void)
{
	/* Memory power is external to the device and the kernel SOF
	 * loader doesn't zero it, so do it ourselves to prevent
	 * possible pollution from previous runs. */
	memset(_mtk_adsp_sram_end, 0, SRAM_END - (uint32_t)&_mtk_adsp_sram_end);
	memset(_mtk_adsp_dram_end, 0, DRAM_END - (uint32_t)&_mtk_adsp_dram_end);
}
