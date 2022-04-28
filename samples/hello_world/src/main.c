/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>

/* How big a stack array should we write to produce cache flush overhead? */
#define DIRTY_STACK_BYTES 1024

/* Xtensa software interrupt to use.  Will be specific to CPU
 * instantiation, see your core-isa.h.  On cAVS devices interrupt 4 is
 * a level 2 interrupt (at the same level as most of the hardware)
 */
#define SOFTWARE_INT 4

k_tid_t main_thread;

void (*IDLE_HOOK)(void);

void z_ready_thread(struct k_thread *thread);

/* Must have just one CPU so measurements make sense */
BUILD_ASSERT(CONFIG_MP_NUM_CPUS == 1);

static ALWAYS_INLINE uint32_t ccount(void)
{
        uint32_t val;

        __asm__ volatile ("rsr.CCOUNT %0" : "=r"(val));
        return val;
}

/* Whitebox hook out of the idle thread, called in the same interrupt
 * lock that will be used to enter k_cpu_idle(), so we can
 * safely/atomically flag an interrupt here.
 */
static void idle_hook(void)
{
	/* Make main thread runnable again (scheduler innards here,
	 * can't use k_thread_resume() because that will context
	 * switch and we want to wait for the interrupt).
	 */
	main_thread->base.thread_state &= ~_THREAD_SUSPENDED;
	z_ready_thread(main_thread);

	__asm__ volatile("wsr %0, INTSET" :: "r"(BIT(SOFTWARE_INT)));
}

/* Wire a noop interrupt to an Xtensa software interrupt.  The only
 * purpose here is to make sure we exit the idle thread the "normal"
 * way, via an interrupt return preempting it.
 */
static void int_handler(const void *param)
{
	ARG_UNUSED(param);
	__asm__ volatile("wsr %0, INTCLEAR" :: "r"(BIT(SOFTWARE_INT)));
}

void main(void)
{
	char stack_buf[DIRTY_STACK_BYTES];
	uint32_t t0, t1, dt, worst, tot, count;

	main_thread = k_current_get();

	irq_connect_dynamic(SOFTWARE_INT, 0, int_handler, NULL, 0);

	__asm__ volatile("wsr %0, INTENABLE" :: "r"(BIT(SOFTWARE_INT)));

	while (true) {
		tot = 0;
		worst = 0;
		count = 10000;

		for (int i = 0; i < count; i++) {
			/* Dirty the cached stack memory */
			memset(stack_buf, 0, ARRAY_SIZE(stack_buf));

			IDLE_HOOK = idle_hook;

			t0 = ccount();
			k_thread_suspend(main_thread);
			t1 = ccount();

			dt = t1 - t0;
			worst = MAX(worst, dt);
			tot += dt;
		}
		printk("last %d avg %d worst %d\n", dt, tot / count, worst);
	}
}
