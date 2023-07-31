#include <stdbool.h>
#include "regs.h"

// Be warned: the interface here is poorly understood.  I did the best
// I could to transcribe it (with a little clarification and
// optimization) from the SOF mt8195 source, but without docs this
// needs to be treated with great care.

// Notes:
// * power-on default is 26Mhz, confirmed with a hacked SOF that
//   loads but stubs out the clk code.
// * The original driver has a 13Mhz mode too, but it doesn't work (it
//   hits all the same code and data paths as 26MHz and acts as a
//   duplicate.
// * The magic numbers in the pll_con2 field are from the original
//   source.  No docs on the PLL register interface are provided.

const struct { unsigned short mhz; bool pll; unsigned int pll_con2; } freqs[] = {
    {  26, false, 0 },
    { 370,  true, 0x831c7628 },
    { 540,  true, 0x8214c4ed },
    { 720,  true, 0x821bb13c },
};

static int cur_idx;

// Can't use CPU-counted loops when changing CPU speed, and don't have
// an OS timer driver yet.  Use the timer hardware directly (ostimer
// is never disabled AFAICT, there's not even an interface for a
// disable bit defined)
static inline void delay_us(int us)
{
    unsigned int t0 = OSTIMER64.cur_l;
    while(OSTIMER64.cur_l - t0 < (us * 13));
}

static void set_pll_power(bool on)
{
    if (on) {
        MTK_CK_CG &= ~MTK_CK_CG_SW;
        MTK_PLL_CTRL.con4 |= MTK_PLL_CON4_PWR_ON;
        delay_us(1);
        MTK_PLL_CTRL.con4 &= ~MTK_PLL_CON4_ISO_EN;
        delay_us(1);
        MTK_PLL_CTRL.con0 |= MTK_PLL_CON0_EN;
        delay_us(20);
    } else {
        MTK_PLL_CTRL.con0 &= ~MTK_PLL_CON0_EN;
        delay_us(1);
        MTK_PLL_CTRL.con4 |= MTK_PLL_CON4_ISO_EN;
        delay_us(1);
        MTK_PLL_CTRL.con4 &= ~MTK_PLL_CON4_PWR_ON;
    }
}

// Oddball utility.  There is a giant array of clocks (of which SOF
// only touches two), each with "clear" and "set" registers which are
// used to set 4-bit fields at a specific offset.  After that, a
// particular bit in one of the "update" registers must be written,
// presumably to latch the input.
static void setclk(int clk, int shift, int updreg, int ubit, int val)
{
    MTK_CLK_GEN.clk_cfg[clk].clr = (0xf << shift);
    if (val) {
        MTK_CLK_GEN.clk_cfg[clk].set = (val << shift);
    }
    MTK_CLK_GEN.update[updreg] = BIT(ubit);
}

#define SETCLK22(val) setclk(22, 0, 2, 24, (val))
#define SETCLK28(val) setclk(28, 16, 3, 18, (val))

void set_cpu_freq(int idx)
{
    if (idx == cur_idx) {
        return;
    }

    if (freqs[idx].pll) {
        // Switch to PLL from 26Mhz
        set_pll_power(true);
        SETCLK22(MTK_CLK22_SEL_PLL);
        SETCLK28(MTK_CLK28_SEL_PLL);
        MTK_PLL_CTRL.con2 = freqs[idx].pll_con2;
    } else {
        // Switch to 26Mhz from PLL
        SETCLK28(MTK_CLK28_SEL_26M);
        SETCLK22(MTK_CLK22_SEL_26M);
        set_pll_power(false);
    }

    cur_idx = idx;
}
