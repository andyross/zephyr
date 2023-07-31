static inline unsigned int ccount(void)
{
    unsigned int t;
    __asm__ volatile("rsr %0, CCOUNT" : "=r"(t));
    return t;
}

static inline void delay(unsigned int cyc)
{
    unsigned int t0 = ccount();
    while(ccount() - t0 < cyc);
}

#define BIT(n) (1<<(n))

////////////////////////////////////////////////////////////////////////
// Interrupt Controller

// Note: strides are different for the two controllers!  The ENABLE
// and STATUS words work like you'd expect, with a one bit meaning
// "enabled" and "active" respectively.  Presumably the other two
// registers are polarity and level- (as in "not edge") triggered?
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

// Transcribed device interrupts on these controllers.  All usable
// interrupts seem to have names, but SOF only uses timers and mbox,
// so YMMV.

#define IRQ1_CQDMA0_MASK    BIT(0)
#define IRQ1_CQDMA1_MASK    BIT(1)
#define IRQ1_CQDMA2_MASK    BIT(2)
#define IRQ1_CQDMA3_MASK    BIT(3)
#define IRQ1_UART_MASK      BIT(4)
#define IRQ1_AFE_MASK       BIT(5)
#define IRQ1_MCU_MASK       BIT(6)
#define IRQ1_I2C4_MASK      BIT(7)
#define IRQ1_I2C5_MASK      BIT(8)
#define IRQ1_RSVD1_MASK     BIT(9)
#define IRQ1_RSVD2_MASK     BIT(10)
#define IRQ1_ASRC0_MASK     BIT(11)
#define IRQ1_ASRC1_MASK     BIT(12)
#define IRQ1_ASRC2_MASK     BIT(13)
#define IRQ1_ASRC3_MASK     BIT(14)
#define IRQ1_ASRC4_MASK     BIT(15)
#define IRQ1_ASRC5_MASK     BIT(16)
#define IRQ1_ASRC6_MASK     BIT(17)
#define IRQ1_ASRC7_MASK     BIT(18)
#define IRQ1_ASRC8_MASK     BIT(19)
#define IRQ1_ASRC9_MASK     BIT(20)
#define IRQ1_ASRC10_MASK    BIT(21)
#define IRQ1_ASRC11_MASK    BIT(22)
#define IRQ1_ASRC12_15_MASK BIT(23)
#define IRQ1_SPM_MASK       BIT(24)
#define IRQ1_SCP_MASK       BIT(25)

#define IRQ23_MBOX0_MASK      BIT(0)
#define IRQ23_MBOX1_MASK      BIT(1)
#define IRQ23_MBOX2_MASK      BIT(2)
#define IRQ23_MBOX3_MASK      BIT(3)
#define IRQ23_MBOX4_MASK      BIT(4)
#define IRQ23_MISC_NNA00_MASK BIT(5)
#define IRQ23_MISC_NNA01_MASK BIT(6)
#define IRQ23_MISC_NNA02_MASK BIT(7)
#define IRQ23_MISC_NNA10_MASK BIT(8)
#define IRQ23_MISC_NNA11_MASK BIT(9)
#define IRQ23_MISC_NNA12_MASK BIT(0)
#define IRQ23_OSTIMER0_MASK   BIT(11)
#define IRQ23_OSTIMER1_MASK   BIT(12)
#define IRQ23_OSTIMER2_MASK   BIT(13)
#define IRQ23_OSTIMER3_MASK   BIT(14)
#define IRQ23_PTIMER_MASK     BIT(15)

////////////////////////////////////////////////////////////////////////
// Timers.  This device has a LOT of timer hardware.  There are SIX
// instantiated devices, with THREE different interfaces!  Not
// including the three Xtensa CCOUNT timers!
//
// In practice only "ostimer0" is used as an interrupt source by the
// original SOF code, and the "ostimer64" and "platform" timers
// reflect the same underlying clock (though they're different
// counters with different values), so other features are hard to
// puzzle out.  The platform timer seems to have a comparator, for
// example, but I don't know how to get it to flag an interrupt.

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

#define OSTIMER_CON_ENABLE     BIT(0)
#define OSTIMER_CON_CLKSRC_MASK 0x30
#define OSTIMER_CON_CLKSRC_32K  0x00 // 32768 Hz
#define OSTIMER_CON_CLKSRC_26M  0x10 // 26 MHz
#define OSTIMER_CON_CLKSRC_BCLK 0x20 // CPU speed, 720 MHz
#define OSTIMER_CON_CLKSRC_PCLK 0x30 // ~312 MHz, maybe 1/3 of a 939 MHz cpu clock in scaling_available_frequencies?

#define OSTIMER_IRQ_ACK_ENABLE BIT(4) // read = status, write = enable
#define OSTIMER_IRQ_ACK_CLEAR  BIT(5)

#define PTIMER (*(volatile struct mtk_platform_timer *)0x10800000)

#define PTIMER_CR_ENABLE         1
#define PTIMER_CR_CLKSRC_MASK 0x30
#define PTIMER_CR_CLKSRC_13M  0x10 // what do clocks 0, 2 and 3 do?

////////////////////////////////////////////////////////////////////////
// Mailbox

// Simple interrupt source.  Each direction has a 5-bit command
// register and will latch an interrupt if any of the bits are
// non-zero.  The interrupt bits get cleared/acknowledged by writing
// ones to the corresponding bits of "cmd_clear".  There are five
// scratch registers for use as message data in each direction.
//
// The same device is mapped at the same address by the host and DSP,
// and the naming is from the perspective of the DSP: the "in"
// registers control interrupts on the DSP, the "out" registers are
// for transmitting data to the host.
//
// There is an array of the devices.  Linux's device-tree defines two.
// SOF uses those for IPC, but also implements platform_trace_point()
// using the third (no linux driver though?).  The upstream headers
// list interrupts for FIVE, and indeed those all seem to be present
// and working.
//
// In practice: The first device (mbox0) is for IPC commands in both
// directions.  The cmd register is written with a 1 ("IPI_OP_REQ")
// and the command is placed in shared DRAM.  The message registers
// are ignored.  The second device (mbox1) is for responses to IPC
// commands, writing a 2 (IPI_OP_RSP) to the command register.  (Yes,
// this is redundant, and the actual value is ignored by the ISRs on
// both sides).

struct mtk_mbox {
    unsigned int in_cmd;
    unsigned int in_cmd_clr;
    unsigned int in_msg[5];
    unsigned int out_cmd;
    unsigned int out_cmd_clr;
    unsigned int out_msg[5];
};

// There are an array of these with a 4k stride.  SOF uses three
// (interrupts in, interrupts out, and the in_msg[0] field of the
// third for trace), Linux only touches the first two.
#define MTK_MBOX(n) (*(volatile struct mtk_mbox *)(0x10816000 + (0x1000 * (n))))

////////////////////////////////////////////////////////////////////////
// CPU Clock Control

struct mtk_pll_control {
    unsigned int con0;
    unsigned int con1;
    unsigned int con2;
    unsigned int con3;
    unsigned int con4;
};

#define MTK_PLL_CTRL (*(volatile struct mtk_pll_control *)0x1000c7e0)

#define MTK_PLL_CON0_BASE_EN BIT(0)
#define MTK_PLL_CON0_EN      BIT(9)
#define MTK_PLL_CON4_ISO_EN  BIT(1)
#define MTK_PLL_CON4_PWR_ON  BIT(0)

struct mtk_clk_gen {
    unsigned int mode;
    unsigned int update[4];
    unsigned int _unused[3];
    struct {
        unsigned int cur;
        unsigned int set;
        unsigned int clr;
    } clk_cfg[29];
};

#define MTK_CLK_GEN (*(volatile struct mtk_clk_gen *)0x10000000)

#define MTK_CLK22_SEL_PLL 8
#define MTK_CLK22_SEL_26M 0

#define MTK_CLK28_SEL_PLL 7
#define MTK_CLK28_SEL_26M 0

#define MTK_CK_CG (*(volatile unsigned int *)0x10720180)

#define MTK_CK_CG_SW 1

