#include "regs.h"
#include "mtprintf.h"

void timer_test(void);

__asm__(".align 4\n\t"
        ".global _start\n\t"
        "_start:\n\t"
        "  movi  a0, 0x0002f\n\t" // was 4002f, but WOE=0 here for CALL0
        "  wsr   a0, PS\n\t"
        "  movi  a0, 0\n\t"
        "  wsr   a0, WINDOWBASE\n\t"
        "  movi  a0, 1\n\t"
        "  wsr   a0, WINDOWSTART\n\t"
        "  rsync\n\t"
        "  movi  a1, 0x40040000\n\t"
        "  j c_main\n\t"); // was call4, but this is a CALL0 image

void fa(void);
void fb(void);
void fc(void);
void fd(void);
void fe(void);

static void enable_mpu(void)
{
    static const unsigned int mpu[][2] = {
        { 0x00000000, 0x06000 }, /* inaccessible null region */
        { 0x10000000, 0x06f00 }, /* all MMIO registers in this region (maybe) */
        { 0x1d000000, 0x06000 }, /* inaccessible */
        { 0x40000000, 0xf7f00 }, /* 256k cached SRAM */
        { 0x40040000, 0x06000 }, /* inaccessible */
        { 0x60000000, 0x06f00 }, /* 17M uncached DRAM */
        { 0x61100000, 0x06f00 }, /* inaccessible */
    };

    // Must write BACKWARDS FROM THE END to avoid introducing a
    // non-monotonic segment at the current instruction fetch.  The
    // exception triggers even if all the segments involved are
    // disabled!
    int nseg = sizeof(mpu)/sizeof(mpu[0]);
    for (int i = 31; i >= 32 - nseg; i--) {
        int mpuidx = i - (32 - nseg);
        unsigned int addren = mpu[mpuidx][0] | 1;
        unsigned int segprot = (mpu[mpuidx][1]) | i;

        // If an existing instruction fetch is in the same segment,
        // wptlb must be preceded by a memw in the same cache line.
        __asm__ volatile("j 1f\n"
                         ".align 8\n"
                         "1:\n"
                         "memw\n"
                         "wptlb %1, %0"
                         :: "r"(addren), "r"(segprot));
    }
}

static void probe_mem(void)
{
    // Probe all the possible tlb pages to intuit ranges
    // (note this takes a second or so and doesn't appear instantly)
    mtprintf("Probing memory map:\n");
    unsigned int addr, tlb, lasttlb = 0xffffffff;
    for(addr = 0; addr < 0xffff0000; addr += 4096) {
        __asm__ volatile("pptlb %0, %1" : "=r"(tlb) : "r"(addr));
        if(tlb != lasttlb) {
            mtprintf(" segment @ 0x%x: access 0x%x type 0x%x\n",
                     addr, (tlb >> 8) & 0xf, (tlb>>12) & 0x1ff);
        }
        lasttlb = tlb;
    }
}

static inline unsigned int ccount(void)
{
    int t;
    __asm__ volatile("rsr %0, CCOUNT" : "=r"(t));
    return t;
}

static void memperf(unsigned int *addr)
{
    const int reps = 1024;
    unsigned int t0, t1, dummy;

    t0 = ccount();
    for (int i = 0; i < reps; i++) {
        __asm__ volatile("l32i %0, %1, 0x00;"
                         "l32i %0, %1, 0x04;"
                         "l32i %0, %1, 0x08;"
                         "l32i %0, %1, 0x0c;"
                         "l32i %0, %1, 0x10;"
                         "l32i %0, %1, 0x14;"
                         "l32i %0, %1, 0x18;"
                         "l32i %0, %1, 0x1c;"
                         "l32i %0, %1, 0x20;"
                         "l32i %0, %1, 0x24;"
                         "l32i %0, %1, 0x28;"
                         "l32i %0, %1, 0x2c;"
                         "l32i %0, %1, 0x30;"
                         "l32i %0, %1, 0x34;"
                         "l32i %0, %1, 0x38;"
                         "l32i %0, %1, 0x3c;"
                         "l32i %0, %1, 0x40;"
                         "l32i %0, %1, 0x44;"
                         "l32i %0, %1, 0x48;"
                         "l32i %0, %1, 0x4c;"
                         "l32i %0, %1, 0x50;"
                         "l32i %0, %1, 0x54;"
                         "l32i %0, %1, 0x58;"
                         "l32i %0, %1, 0x5c;"
                         "l32i %0, %1, 0x60;"
                         "l32i %0, %1, 0x64;"
                         "l32i %0, %1, 0x68;"
                         "l32i %0, %1, 0x6c;"
                         "l32i %0, %1, 0x70;"
                         "l32i %0, %1, 0x74;"
                         "l32i %0, %1, 0x78;"
                         "l32i %0, %1, 0x7c;" : "=r"(dummy) : "r"(addr));
    }
    t1 = ccount();

    int n = 32 * reps;
    int cyc = t1 - t0;

    // Kludgey fixed point.  Note edge case where it prints
    // e.g. "1.2" instead of "1.02"!!! (No precision specifiers in our
    // printf)
    int ratex100 = (100 * cyc) / n;
    int rate_i = ratex100 / 100;
    int rate_f = ratex100 % 100;
    mtprintf("  %d.%d cyc/read\n", rate_i, rate_f);
}

// Look for holes in DRAM.  Thought I saw one earlier, but this shows
// that the full 17M region is usable.
void dram_walk(void)
{
    const int mark = 0x5a5af123;
    int lastok = 0;
    for (int i = 0x60000000; i < 0x7fffff00; i += 128) {
        volatile int *p = (int *)i;
        int old = *p;
        *p = mark;
        int ok = *p == mark;
        if (!ok && lastok) {
            mtprintf("Write to %p didn't take\n", p);
        } else if(ok && !lastok) {
            mtprintf("Write to %p working\n", p);
        }
        *p = old;
        lastok = ok;
    }
}

void mem_benchmark(void) {
    // FIXME: this is showing artificially slow, as both the
    // instruction fetches and DRAM fetches are uncached.  Would do
    // better to turn on icache separately.
    mtprintf("Benchmarking uncached SRAM\n");
    memperf((void *)0x40000000);

    // The MPU configuration enabled caching at the bus level, but
    // didn't actually enable the cache hardware itself.  Turn it on
    // via MEMCTL.
    unsigned int memctl = 0xffffff00;
    __asm__ volatile("wsr %0, MEMCTL; rsync" :: "r"(memctl));

    mtprintf("Benchmarking (cached?) SRAM\n");
    memperf((void *)0x40000000);
    mtprintf("Benchmarking uncached DRAM\n");
    memperf((void *)0x60000000);
}

void irq_reset(void)
{
    IRQ1_ENABLE = 0;
    IRQ23_ENABLE = 0;

    // Clear out any existing interrupts
    int zeros = 0, ones = -1, intstat;
    __asm__ volatile("wsr %1, INTENABLE;"
                     "wsr %2, INTCLEAR;"
                     "rsync; rsr %0, INTERRUPT" :
                     "=r"(intstat) : "r"(zeros), "r"(ones));

    mtprintf("INTERRUPT was 0x%x\n", intstat);

    // Xtensa timer interrupts need to be cleared by writing to CCOMPAREx
    __asm__ volatile("wsr %0, CCOMPARE0" :: "r"(ones));
    __asm__ volatile("wsr %0, CCOMPARE1" :: "r"(ones));
    __asm__ volatile("wsr %0, CCOMPARE2" :: "r"(ones));

    // Finally enable the external device interrupts 1 and 23, which
    // were masked above by their own enable registers
    int mask = (1<<1) | (1<<23);
    __asm__ volatile("wsr %0, INTENABLE" :: "r"(mask));

    __asm__ volatile("rsync; rsr %0, INTERRUPT" : "=r"(intstat));
    mtprintf("INTERRUPT now 0x%x\n", intstat);
}

void c_main(void)
{
    // Clear DRAM (effects "MTPRINTF_LEN = 0;" too so logging works)
    for (int i = 0x60000000; i < 0x61100000; i+=4) {
        if(i & 0x3) continue;
        *(int*)i = 0;
    }
    mtprintf("Hello, world!\n");

    // Clear SRAM.  Note that this clobbers our own stack, but it's
    // just a quick hack and as I read the generated code is safe as
    // long as no local variables have been set previously.
    extern char img_sram_end[];
    mtprintf("sram end @ %p\n", &img_sram_end);
    for (int i = (int)&img_sram_end; i < 0x40040000; i+=4) {
        if(i & 0x3) continue;
        *(int*)i = 0;
    }

    enable_mpu();

    irq_reset();

    probe_mem();

    //dram_walk();

    timer_test();

    mem_benchmark();

    // Validate that printf is working, also try some deep-ish
    // recursion to make sure windows don't go wonky.
    mtprintf("%x\n", 0x1234567);
    mtprintf("string: %s\n", "ok");
    mtprintf("decimal: %d\n", 9);
    fa();

    while(1);
}

void fa(void) { mtprintf("%s\n", __func__); fb(); }
void fb(void) { mtprintf("%s\n", __func__); fc(); }
void fc(void) { mtprintf("%s\n", __func__); fd(); }
void fd(void) { mtprintf("%s\n", __func__); fe(); }
void fe(void) { mtprintf("%s\n", __func__);       }

