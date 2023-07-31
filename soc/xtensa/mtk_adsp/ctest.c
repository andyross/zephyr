#include "regs.h"
#include "mtprintf.h"

#ifndef __XCC__
// Zephyr SDK generates memset() calls, but has no library for __builtin_memset?
#include <string.h>
void *memset(void *s, int c, size_t n)
{
    for (int i = 0; i < n; i++) ((char *)s)[i] = c;
    return s;
}
#endif

void timer_test(void);
void set_cpu_freq(int idx);
void ostimer_bench(void);

__asm__(".align 4\n\t"
        ".global _start\n\t"
        "_start:\n\t"
        "  movi  a0, 0x4002f\n\t" // 40000=WOE, 2=EXCM, F=INTLVL
        "  wsr   a0, PS\n\t"
        "  movi  a0, 0\n\t"
        "  wsr   a0, WINDOWBASE\n\t"
        "  movi  a0, 1\n\t"
        "  wsr   a0, WINDOWSTART\n\t"
        "  rsync\n\t"
        "  movi  a1, 0x40040000\n\t"
        "  call4 c_main\n\t"); // call4 or j, depending on WOE

// MPU TLB opcodes don't build wiht gcc, need a solution
static void enable_mpu(void)
{
#ifdef __XCC__
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
#endif
}

static void probe_mem(void)
{
#ifdef __XCC__
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
#endif
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

    // Kludgey fixed point.  Note edge case to keep it from printing
    // e.g. "1.2" instead of "1.02" (No precision specifiers in our
    // printf)
    int ratex100 = (100 * cyc) / n;
    int rate_i = ratex100 / 100;
    int rate_f = ratex100 % 100;
    mtprintf("  %d.%s%d cyc/read\n", rate_i, (rate_f && rate_f < 10) ? "0" : "", rate_f);
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
    // We're starting with caching enabled at the MPU level, but the
    // caches themselves aren't active yet, that's happens via the
    // MEMCTL SR.

    mtprintf("Benchmarking uncached SRAM (incl. instruction fetches!)\n");
    memperf((void *)0x40000000);

    // Per my reading of the ISA spec, this enables icache but not dcache
    unsigned int memctl = (0xffffffff << 18);
    __asm__ volatile("wsr %0, MEMCTL; rsync" :: "r"(memctl));

    mtprintf("Benchmarking SRAM with icache enabled\n");
    memperf((void *)0x40000000);

    // Now enable all caches
    memctl = 0xffffff00;
    __asm__ volatile("wsr %0, MEMCTL; rsync" :: "r"(memctl));

    mtprintf("Benchmarking cached SRAM\n");
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

void mbox_test(void)
{
    // Write to the out message fields on all five mbox devices to be
    // sure they read back the same data. (FWIW: touching the "sixth"
    // one panics, so there are indeed five)
    for (int i = 0; i < 5; i++) {
        for(int j = 0; j < 5; j++) {
            MTK_MBOX(i).out_msg[j] = (i << 8) | j;
        }
        mtprintf("mbox%d in msg: { 0x%x, 0x%x, 0x%x, 0x%x, 0x%x }\n", i,
                 MTK_MBOX(i).in_msg[0], MTK_MBOX(i).in_msg[1], MTK_MBOX(i).in_msg[2],
                 MTK_MBOX(i).in_msg[3], MTK_MBOX(i).in_msg[4]);
        mtprintf("mbox%d out msg: { 0x%x, 0x%x, 0x%x, 0x%x, 0x%x }\n", i,
                 MTK_MBOX(i).out_msg[0], MTK_MBOX(i).out_msg[1], MTK_MBOX(i).out_msg[2],
                 MTK_MBOX(i).out_msg[3], MTK_MBOX(i).out_msg[4]);
    }

    // Signal the SOF host CPU interrupts for giggles.  The SOF driver
    // synchronously produces errors for each.  Very interestingly,
    // and counter to the SOF headers, there seem to be FIVE
    // significant bits in the CMD register.  Writing an 0x10 triggers
    // an interrupt, but 0x20 doesn't.
    MTK_MBOX(0).out_cmd = 0x10;
    MTK_MBOX(1).out_cmd = 0x8;

    // Now signal our own interrupts to be sure they flag.  They do,
    // though if you read carefully mbox0/1 tend to be polluted with
    // incoming events from the host.
    for (int i = 0; i < 5; i++) {
        MTK_MBOX(i).in_cmd_clr = MTK_MBOX(i).in_cmd; // clear interrupt
        mtprintf("mbox%d cmd 0x%x irq23 0x%x\n", i, MTK_MBOX(i).in_cmd, IRQ23_STATUS);
        IRQ23_ENABLE |= (IRQ23_MBOX0_MASK << i);

        MTK_MBOX(i).in_cmd = 0x10; // flag interrupt
        mtprintf("mbox%d cmd 0x%x irq23 0x%x\n", i, MTK_MBOX(i).in_cmd, IRQ23_STATUS);
        MTK_MBOX(i).in_cmd_clr = MTK_MBOX(i).in_cmd; // clear again
    }
}

// Function defined in timer.c to defeat optimization
int (*rec_fn_ptr)(int);
int rec_func(int arg);

void reg_win_test(void)
{
    // Deep recursion test
    rec_fn_ptr = rec_func;
    rec_fn_ptr(64);

    // Validate that printf is working, varargs are a good exercise of
    // stack management
    mtprintf("%x\n", 0x1234567);
    mtprintf("string: %s\n", "ok");
    mtprintf("decimal: %d\n", 9);
}

void c_main(void)
{
    // Clear DRAM (effects "MTPRINTF_LEN = 0;" too so logging works)
    for (int i = 0x60000000; i < 0x61100000; i+=4) {
        if(i & 0x3) continue;
        *(int*)i = 0;
    }
    mtprintf("Hello, world! [" _DATE "]\n");

    // Clear SRAM.  Leave 4k for our active stack
    extern char img_sram_end[];
    mtprintf("sram end @ %p\n", &img_sram_end);
    for (int i = (int)&img_sram_end; i < 0x4003e000; i++) {
        *(char*)i = 0;
    }

    // Set up vector table
    extern char z_xtensa_vecbase;
    __asm__ volatile("wsr %0, VECBASE; rsync" :: "r"(&z_xtensa_vecbase));

    // Enable exceptions
    unsigned int ps = 0x4000f;
    __asm__ volatile("wsr %0, PS; rsync" :: "r"(ps));

    // Clock state is an external device, and frustratingly can't
    // currently be read from the register state for lack of docs.
    // Hardware power up is 26 MHz, but SOF leaves it at 720, and our
    // test code might have mucked with something else.  Set a
    // non-zero mode to effect a change (we just zeroed the "current"
    // index!).  Mode 3 is 720 MHz
    set_cpu_freq(3);

    reg_win_test();

    enable_mpu();

    irq_reset();

    probe_mem();

    //dram_walk();

    timer_test();

    mem_benchmark();

    mbox_test();

    // Enumerate and benchmark all the CPU speeds
    for(int i = 0; i < 4; i++) {
        mtprintf("setting cpu freq %d...\n", i);
        set_cpu_freq(i);
        ostimer_bench();
    }

#if 0
    // Validate cycle time vs. realtime (720 MHz is indeed correct,
    // though the output of the python script is jankier than I'd
    // like...)
    for(int i = 0; i < 20; i++) {
        mtprintf("%d...\n", i);
        delay(720000000);
    }
#endif

    mtprintf("finis\n");
    while(1);
}


