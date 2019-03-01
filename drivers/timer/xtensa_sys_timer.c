/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <drivers/system_timer.h>
#include <sys_clock.h>
#include <spinlock.h>
#include <xtensa_rtos.h>

#define TIMER_IRQ UTIL_CAT(XCHAL_TIMER,		\
			   UTIL_CAT(CONFIG_XTENSA_TIMER_ID, _INTERRUPT))

#define CYC_PER_TICK (CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC	\
		      / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_TICKS ((0xffffffffu - CYC_PER_TICK) / CYC_PER_TICK)
#define MIN_DELAY 1000

static struct k_spinlock lock;
static volatile unsigned int last_count;

#ifdef CONFIG_QEMU_TARGET
static volatile unsigned int expect_expire;
#endif

static void set_ccompare(u32_t val)
{
	__asm__ volatile ("wsr.CCOMPARE" STRINGIFY(CONFIG_XTENSA_TIMER_ID) " %0"
			  :: "r"(val));
}

static u32_t ccount(void)
{
	u32_t val;

	__asm__ volatile ("rsr.CCOUNT %0" : "=r"(val));
	return val;
}

static void ccompare_isr(void *arg)
{
	ARG_UNUSED(arg);

	k_spinlock_key_t key = k_spin_lock(&lock);
	u32_t curr = ccount();

#ifdef CONFIG_QEMU_TARGET
	/* Qemu exposes a real-world clock time to us, which is useful
	 * for most things, but has the side effect that, when the
	 * qemu process is descheduled by the host OS (for example,
	 * during a big loaded test run) the guest appears to "time
	 * warp" ahead and interrupts can arrive very late.  Assume
	 * that any interrupt arriving later than half a tick short of
	 * the expected time was "on time" and skew our internal clock
	 * references to reflect that.
	 */
	if ((int)(curr - expect_expire) > -(CYC_PER_TICK / 2)) {
		last_count = curr - (expect_expire - last_count);
		expect_expire = curr + 0x7fffffff;
	}
#endif

	u32_t dticks = (curr - last_count) / CYC_PER_TICK;

	last_count += dticks * CYC_PER_TICK;

	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		u32_t next = last_count + CYC_PER_TICK;

		if ((s32_t)(next - curr) < MIN_DELAY) {
			next += CYC_PER_TICK;
		}
		set_ccompare(next);
	}

	k_spin_unlock(&lock, key);
	z_clock_announce(IS_ENABLED(CONFIG_TICKLESS_KERNEL) ? dticks : 1);
}

/* The legacy Xtensa platform code handles the timer interrupt via a
 * special path and must find it via this name.  Remove once ASM2 is
 * pervasive.
 */
#ifndef CONFIG_XTENSA_ASM2
void _timer_int_handler(void *arg)
{
	return ccompare_isr(arg);
}
#endif

int z_clock_driver_init(struct device *device)
{
	IRQ_CONNECT(TIMER_IRQ, 0, ccompare_isr, 0, 0);
	set_ccompare(ccount() + CYC_PER_TICK);
	irq_enable(TIMER_IRQ);
	return 0;
}

void z_clock_set_timeout(s32_t ticks, bool idle)
{
	ARG_UNUSED(idle);

#if defined(CONFIG_TICKLESS_KERNEL)
	ticks = ticks == K_FOREVER ? MAX_TICKS : ticks;
	ticks = MAX(MIN(ticks - 1, (s32_t)MAX_TICKS), 0);

	k_spinlock_key_t key = k_spin_lock(&lock);
	u32_t curr = ccount(), cyc;

	/* Round up to next tick boundary */
	cyc = ticks * CYC_PER_TICK + (curr - last_count) + (CYC_PER_TICK - 1);
	cyc = (cyc / CYC_PER_TICK) * CYC_PER_TICK;
	cyc += last_count;

	if ((cyc - curr) < MIN_DELAY) {
		cyc += CYC_PER_TICK;
	}

#ifdef CONFIG_QEMU_TARGET
	expect_expire = cyc;
#endif

	set_ccompare(cyc);
	k_spin_unlock(&lock, key);
#endif
}

u32_t z_clock_elapsed(void)
{
	if (!IS_ENABLED(CONFIG_TICKLESS_KERNEL)) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	u32_t ret = (ccount() - last_count) / CYC_PER_TICK;

	k_spin_unlock(&lock, key);
	return ret;
}

u32_t _timer_cycle_get_32(void)
{
	return ccount();
}

#ifdef CONFIG_SMP
void smp_timer_init(void)
{
	set_ccompare(ccount() + CYC_PER_TICK);
	irq_enable(TIMER_IRQ);
}
#endif
