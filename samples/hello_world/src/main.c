#include <zephyr.h>
#include <xtensa/config/core-isa.h>

/* Make sure we're testing what we think we're testing!
 *
 * This wants:
 *   CONFIG_SMP=y
 *   CONFIG_MP_NUM_CPUS=1
 *   CONFIG_KERNEL_COHERENCE=y
 *   CONFIG_THREAD_STACK_INFO=y
 */
BUILD_ASSERT(IS_ENABLED(CONFIG_SMP));
BUILD_ASSERT(CONFIG_MP_NUM_CPUS == 1);
BUILD_ASSERT(!IS_ENABLED(CONFIG_ARCH_HAS_COHERENCE) ||
	     IS_ENABLED(CONFIG_KERNEL_COHERENCE));

/* Xtensa interrupt number we're going to trigger */
#ifdef CONFIG_BOARD_QEMU_XTENSA
#define SOFT_IRQ 7
#else
#define SOFT_IRQ 0
#endif

K_SEM_DEFINE(sem, 0, 1);

void fill_windows(void);

uint32_t stack_copy[512];
size_t stack_copy_size;
size_t main_stack_top;
uint32_t *main_sp;

void soft_isr(const void *p)
{
	/* The ISR does an absolute minimum amount of work, to avoid
	 * spilling any register state from the interrupted caller
	 */
	k_sem_give(&sem);
}

void thread_fn(void *p1, void *p2, void *p3)
{
	printk("thread: starting\n");

	/* We are high priority and run first.  Go to sleep waiting to
	 * be awoken by the ISR, itself triggered from the main
	 * thread.
	 */
	k_sem_take(&sem, K_FOREVER);

#ifdef CONFIG_SOC_FAMILY_INTEL_ADSP
	/* Back from the interrupt, in a different thread.  Quickly
	 * invalidate the main stack's thread area.  It SHOULD have
	 * been spilled already, making this a noop.  But if it wasn't
	 * we don't want our calls here to accidentally flush the
	 * lines and hide a bug.
	 */
	z_xtensa_cache_inv(main_sp, stack_copy_size*sizeof(int));
#endif
	/* The main thread stack should have been spilled by the
	 * interrupt. Check it's contents vs. what it saved for us to
	 * see if something didn't get saved properly.
	 */
	printk("    addr        stack mem   copy\n");
	printk("    ------      ----------  ----------\n");
	int *stack = (int *)(main_stack_top - sizeof(int) * stack_copy_size);


#ifdef CONFIG_SOC_FAMILY_INTEL_ADSP
	stack = z_soc_uncached_ptr(stack);
#endif

	for(int i = stack_copy_size - 1; i >= 0; i--) {
		printk("%3d 0x%8.8x  0x%8.8x  0x%8.8x %s\n",
		       i, (size_t)&stack[i], stack[i], stack_copy[i],
		       (stack[i] == stack_copy[i]) ? "" : "***");
	}

	printk("thread: end\n");
}

K_THREAD_DEFINE(thread, 2048, thread_fn, NULL, NULL, NULL, -1, 0, 0);

/* This runs in the main thread immediately before the interrupt.  The
 * register windows have been filled by our callers.  We have to be
 * careful that nothing issues a call here lest it mess up the window
 * state!
 */
void launch_switch(void)
{
	uint32_t *sp;
	__asm__ volatile("mov %0, a1" : "=r"(sp));

	int windowbase, windowstart;
	__asm__ volatile("rsr %0, WINDOWBASE; rsr %1, WINDOWSTART"
			 : "=r"(windowbase), "=r"(windowstart));

	/* Verify that we have a 1 bit at WINDOWSTART[WINDOWBASE]
	 * representing this current call.  The next 0-3 quads may be
	 * 1 or 0 depending on how the compiler generates this
	 * function (they represent our local registers A4-A15, which
	 * the compiler may choose to use or not), so ignore them.
	 * All other bits should be 1, indicating a "full" register
	 * window state.
	 */
	int nwin = XCHAL_NUM_AREGS / 4;
	__ASSERT(windowstart & BIT(windowbase), "");
	for (int off = 4; off < nwin; off++) {
		__ASSERT(windowstart & BIT((windowbase + off) % nwin), "");
	}

	/* Make a copy of our own stack in uncached memory,
	 * which will have the effect of both saving it for checking
	 * later and reliably populating it in our own L1 cache
	 */
	main_sp = sp;
	stack_copy_size = (main_stack_top - (size_t)main_sp) / sizeof(int);
	for(int i = 0; i < stack_copy_size; i++) {
		stack_copy[i] = main_sp[i];
	}

#ifdef CONFIG_SOC_FAMILY_INTEL_ADSP
	/* Now, carefully wipe the SRAM memory underneath our cache
	 * with a recognizable pattern, so we can see what got
	 * spilled.  Be very careful not to touch anything that might
	 * populate a new cache line and accidentally spill
	 * prematurely.
	 */
	int *usp = z_soc_uncached_ptr(main_sp);
	for(int i = 0; i < stack_copy_size; i++) {
		usp[i] = 0xcab00d1e;
	}
#endif

	/* Finally, trigger the interrupt */
	int imask = 1 << SOFT_IRQ;
	IRQ_CONNECT(SOFT_IRQ, 0, soft_isr, NULL, 0);
	__asm__ volatile("wsr %0, INTENABLE; rsync" :: "r"(imask));
	__asm__ volatile("wsr %0, INTSET; rsync" :: "r"(imask));
}

void main(void)
{
	uint32_t *sp;
	__asm__ volatile("mov %0, a1" : "=r"(sp));
	printk("main: starting (sp %p)\n", sp);

	k_tid_t th = k_current_get();
	main_stack_top = th->stack_info.start + th->stack_info.size;
	printk("main_stack_top 0x%x\n", main_stack_top);

	fill_windows();
	printk("main: end\n");
}

/* Assembly hackery to fill the register window state with known(ish)
 * values so we can see when they get spilled, then calls
 * launch_switch().  Sixteen frames is enough to guarantee that
 * WINDOWBASE will be all 1's at the entry to launch_switch().
 */
#define __DO_CALL4(n)	      \
	".align 4         \n" \
	"1:               \n" \
	"  entry a1, 16   \n" \
	"  movi a2, " #n "\n" \
	"  movi a3, " #n "\n" \
	"  call4 1f       \n" \
	"  retw           \n"

__asm__(".global fill_windows  \n"
	".align 4              \n"
	"fill_windows:         \n"
	"  entry a1, 16        \n"
	"  movi a2, 0          \n"
	"  movi a3, 0          \n"
	"  call4 1f            \n"
	"  retw                \n"
	__DO_CALL4(1)
	__DO_CALL4(2)
	__DO_CALL4(3)
	__DO_CALL4(4)
	__DO_CALL4(5)
	__DO_CALL4(6)
	__DO_CALL4(7)
	__DO_CALL4(8)
	__DO_CALL4(9)
	__DO_CALL4(10)
	__DO_CALL4(11)
	__DO_CALL4(12)
	__DO_CALL4(13)
	__DO_CALL4(14)
	".align 4              \n"
	"1:                    \n"
	"  entry a1, 16        \n"
	"  movi a2, 15         \n"
	"  movi a3, 15         \n"
	"  call4 launch_switch \n"
	"  retw                \n"
	);
