////////////////////////////////////////////////////////////////////////
// Interrupt Controller

// Note: strides are different for the two controlers!  The ENABLE and
// STATUS words work like you'd expect, with a one bit meaning
// "enabled" and "active" respectively.  Presumably the other two
// registers are POLarity and level (as in "not edge") triggered?
// They aren't used by legacy SOF code.

// Per the code, this has 26 valid inputs masked with 0x3ffffff0
#define IRQ1_POL    (*(volatile unsigned int *)0x10680120)
#define IRQ1_ENABLE (*(volatile unsigned int *)0x10680130)
#define IRQ1_LEVEL  (*(volatile unsigned int *)0x10680140)
#define IRQ1_STATUS (*(volatile unsigned int *)0x10680150)

// Likewise only bits 0xffff are valid per upstream code
#define IRQ23_POL    (*(volatile unsigned int *)0x108030f0)
#define IRQ23_ENABLE (*(volatile unsigned int *)0x108030f4)
#define IRQ23_LEVEL  (*(volatile unsigned int *)0x108030f8)
#define IRQ23_STATUS (*(volatile unsigned int *)0x108030fc)

// Known device interrupts on these controllers (the original SOF
// source is a little confusing with its naming, and only actually
// uses two mbox interrupts and one ostimer, so this list is very
// partial and largely untested)

#define IRQ23_OSTIMER0_MASK (1<<11)
#define IRQ23_OSTIMER1_MASK (1<<12)
#define IRQ23_OSTIMER2_MASK (1<<13)
#define IRQ23_OSTIMER3_MASK (1<<14)

////////////////////////////////////////////////////////////////////////
// Timers.  SIX instantiated devices, with THREE different interfaces!
// Not including the three Xtensa architectural timers!

struct mtk_ostimer {
       unsigned int con;
       unsigned int rst;
       unsigned int cur;
       unsigned int irq_ack;
};

struct mtk_ostimer64 {
       unsigned int con;
       unsigned int init_l;
       unsigned int init_h;
       unsigned int cur_l;
       unsigned int cur_h;
       unsigned int tval_h;
       unsigned int irq_ack;
};

struct mtk_platform_timer {
       unsigned int cr;
       unsigned int sr;
       unsigned int cv_l;
       unsigned int cv_h;
       unsigned int wacr;
       unsigned int racr;
};

#define OSTIMER64 (*(volatile struct mtk_ostimer64 *)0x1080d080)

#define OSTIMERS ((volatile struct mtk_ostimer *)0x1080d000)
#define OSTIMER0 (OSTIMERS[0])
#define OSTIMER1 (OSTIMERS[1])
#define OSTIMER2 (OSTIMERS[2])
#define OSTIMER3 (OSTIMERS[3])

#define PTIMER (*(volatile struct mtk_platform_timer *)0x10800000)

