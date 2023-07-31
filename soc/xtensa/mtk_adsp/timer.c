#include "regs.h"
#include "mtprintf.h"

// Transplanted from ctest.c to defeat optimized inlining
int rec_func(int arg)
{
    extern int (*rec_fn_ptr)(int);
    if (arg > 0) {
        // (makes sure that local state gets retained across the call)
        if (arg != rec_fn_ptr(arg - 1)) {
            mtprintf("OOPS!\n");
        }
        return arg + 1;
    }
    mtprintf("bottom\n");
    return arg + 1;
}

unsigned long long now(void)
{
    unsigned int l, h0, h1;

    do {
        h0 = OSTIMER64.cur_h;
        l  = OSTIMER64.cur_l;
        h1 = OSTIMER64.cur_h;
    } while(h0 != h1);

    return (((unsigned long long)h0) << 32) | l;
}

void ostimer_bench(void)
{
    unsigned long long t, t0 = now();
    unsigned int cc, cc0 = ccount();

    do {
        t = now();
        cc = ccount();
    } while(cc - cc0 < 10000000);
    mtprintf("  %d cpu cyc in %d ostimer64 cyc\n", cc - cc0, (int)(t - t0));
}

void ptimer_bench(void)
{
    unsigned int cc, cc0, pt, pt0 = PTIMER.cv_l;

    cc0 = ccount();
    do {
        pt = PTIMER.cv_l;
        cc = ccount();
    } while(cc - cc0 < 10000000);
    mtprintf("  %d cpu cyc in %d platform timer cyc\n", cc - cc0, (int)(pt - pt0));
}

void ostimer1_bench(void)
{
    OSTIMER1.con &= ~1; // Disable
    OSTIMER1.rst = 0xffffffff;
    OSTIMER1.con |= 1;

    unsigned int cc, cc0, pt, pt0 = OSTIMER1.cur;

    cc0 = ccount();
    do {
        pt = OSTIMER1.cur;
        cc = ccount();
    } while(cc - cc0 < 10000000);
    OSTIMER1.con &= ~1; // Disable
    mtprintf("  %d cpu cyc in %d ostimer[n] timeout cyc\n", cc - cc0, (int)(pt0 - pt));
}

static inline unsigned int intsr(void)
{
    unsigned int sr;
    __asm__ volatile("rsr %0, INTERRUPT" : "=r"(sr));
    return sr;
}

// Note that these timers are EXTERNAL hardware to the DSP.  They
// aren't affected by DSP reset, and will have whatever state previous
// firmware (likely SOF, at boot) left them in.
void timer_test(void)
{
    ostimer_bench();
    ptimer_bench();
    //ostimer1_bench();

    // The 32 bit ostimer's are down-counters with reset.  Low bit of
    // con is an enable flag, without which cur reports zero always,
    // and register writes to rst are ignored.  Note that writes to
    // RST automatically set CUR, there's no need to change both.
    OSTIMER0.con &= ~1; // Disable
    OSTIMER0.rst = 1000000;
    OSTIMER0.irq_ack |= (1<<5); // clear/ack IRQ
    OSTIMER0.irq_ack |= (1<<4); // enable IRQ
    OSTIMER0.con |= 1; // Enable

    // And enable the interrupt at the controller (the Xtensa arch IRQ
    // is already enabled)
    IRQ23_ENABLE |= IRQ23_OSTIMER0_MASK;

    // Brief digression into interrupt controller to watch it change:
    mtprintf("(irq state)    SR 0x%x: IRQ1 0x%x  IRQ23 0x%x\n", intsr(), IRQ1_STATUS, IRQ23_STATUS);

    // Watch for the interrupt flag in bit 4 to change, and verify
    // that the time is immediately after the rollover to 1M
    int lastirq = OSTIMER0.irq_ack & (1<<4);
    while(1) {
        int irqsta = OSTIMER0.irq_ack & (1<<4);
        if (irqsta != lastirq) {
            mtprintf("STATUS %d -> %d @ T%d\n", lastirq, irqsta, OSTIMER0.cur);
            break;
        }
        lastirq = irqsta;
    }

    // Interrupts again: note that bit 11 on IRQ23 is now set!
    mtprintf("(irq fired)    SR 0x%x: IRQ1 0x%x  IRQ23 0x%x\n", intsr(), IRQ1_STATUS, IRQ23_STATUS);

    // Clear the interrupt & validate
    //
    // Note that unlike RST, this does not require disabling the device first
    //
    // Also note that this clears the bit on the 2nd-level controller,
    // but the latch on the CPU itself stays active!  That's
    // surprising, since per docs Xtensa external interrupts should be
    // cleared at the device level...  Looks like it's going to demand
    // we service it anyway.
    OSTIMER0.irq_ack |= (1<<5); // clear/ack IRQ
    mtprintf("(irq cleared): SR 0x%x: IRQ1 0x%x  IRQ23 0x%x\n",
             intsr(), IRQ1_STATUS, IRQ23_STATUS);

    // Check OSTIMER rate vs. OSTIMER64.  Note that the 32 bit timer
    // runs at exactly 2x speed!  This seems to be intentional per the
    // SOF driver source, but seems needless.
    OSTIMER0.con &= ~1; // Disable
    OSTIMER0.rst = 0xffffffff;
    OSTIMER0.con |= 1; // Enable
    unsigned int ta0 = OSTIMER0.cur, tb0 = OSTIMER64.cur_l, ta, tb;
    do {
        ta = OSTIMER0.cur;
        tb = OSTIMER64.cur_l;
    } while(tb - tb0 < 1000000);
    mtprintf("%d ostimer64 cycles vs. %d ostimer0 cycles\n", tb-tb0, ta0-ta);

    // Now just watch it for a while to verify by hand that it works
    for (int i = 0; i < 3; i++) {
        mtprintf("OSTIMER0 con 0x%x cur %d rst %d irq 0x%x\n",
                 OSTIMER0.con, OSTIMER0.cur, OSTIMER0.rst, OSTIMER0.irq_ack);
        for(volatile int _l = 0; _l < 100000; _l++);
    }

    // The "platform" timer has a third (!) interface.  This is
    // another 64 bit up-counter.  It appears to use the same
    // underlying clock as ostimer, but the counter value is
    // separately tracked.
    for (int i = 0; i < 3; i++) {
        unsigned int pt = PTIMER.cv_l;
        unsigned int ot = OSTIMER64.cur_l;
        mtprintf("PTIMER.cv_l %d (diff %d vs. ostimer)\n", pt, (int)(pt - ot));
        for(volatile int _l = 0; _l < 100000; _l++);
    }

    // OSTIMER[1] is a safe playground for experimenting with clock sources
    for(int i = 0; i < 4; i++) {
        mtprintf("Benchmarking clock %d on ostimer1\n");
        OSTIMER1.con &= ~1; // Disable
        OSTIMER1.con = (OSTIMER1.con & ~OSTIMER_CON_CLKSRC_MASK) | (i << 4);
        ostimer1_bench();
        OSTIMER1.con &= ~1; // Disable
    }

#if 0
    // This needs more experimentation: the writes to clocksrc don't
    // take, only a value of 1 ever reads back, which is what SOF
    // uses.  Maybe only 26M works?
    for(int i = 0; i < 4; i++) {
        PTIMER.cr &= ~PTIMER_CR_ENABLE;
        PTIMER.cr = (PTIMER.cr & ~PTIMER_CR_CLKSRC_MASK) | (i << 4);
        PTIMER.cr |= PTIMER_CR_ENABLE;
        mtprintf("Benchmarking clock %d PTIMER.cr = 0x%x\n", i, PTIMER.cr);
        ostimer_bench();
        ptimer_bench();
    }
#endif

#if 0
    // Do the same game with fields on ostimer64, which isn't
    // documented to behave the same but might?  (No, it doesn't.  No
    // effect, writes don't take)
    for(int i = 0; i < 4; i++) {
        OSTIMER64.con &= ~1; // Disable
        OSTIMER64.con = (OSTIMER64.con & 0xffffff0f) | (i << 4);
        OSTIMER64.con |= 1; // Disable
        mtprintf("Benchmarking clock %d on ostimer64 (con 0x%x)\n", i, OSTIMER64.con);
        ostimer_bench();
    }
#endif
}
