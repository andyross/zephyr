#include <zephyr/kernel.h>

/*
 * Design Notes and Dirty Trickery:
 *
 * 1. The swap code is intended to be inlined into surrounding C code,
 *    avoiding branch and marshalling code.
 *
 * 2. The entry API here is to place the current/outgoing thread's
 *    callee_saved pointer in r0 and the incoming in r12.  These are
 *    deliberate choices: r0's slot is not needed when STM'ing the top
 *    four words of the exception frame, and r12 survives
 *    that process until needed by the restore step.
 *
 * 3. The r0-r3 and r12 scratch registers are not saved, and are
 *    clobbered with arbitrary stack junk (but importantly: NOT data
 *    from another thread!) when this thread resumes.  The intent is
 *    that we rely on the compiler's register allocator to replace
 *    code to save these registers in the thread context with spills
 *    managed in the outer context, some of which may be elided.
 *
 * 4. The final step needs to atomically restore the stack pointer and
 *    interrupted PC without using GPR state.  That's done with a
 *    straightforward "POP {PC}", but requires that the resumed PC be
 *    the top word of the exception frame, which is normally occupied
 *    by XPSR.  We load/store PC into that spot manually, spending
 *    four cycles to do so.
 *
 * 5. Interrupts are enabled the instant the old context is saved and
 *    the new stack pointer is set.  The restore process is fully
 *    interruptible.
 *
 * 6. No attempt is made to align the stack to 8 bytes here, which is
 *    done by hardware traps in some configurations
 *    (c.f. CONFIG_STACK_ALIGN_DOUBLE_WORD).  Likewise it expects that
 *    any contexts it restores do not (!) have an extra word stuffed
 *    for alignment.
 *
 * Future ideas (and dirtier trickery):
 *
 * + Currently the code to construct the resume address (which is a
 *   constant, it's just the instruction immediately following the
 *   sequence) from the frame requires three cycles: a LDR of the
 *   label address and an ADD to set the thumb bit.  It's not possible
 *   in assembler syntax to combine these two operations (because the
 *   value isn't known until link time).  But it is possible to do it
 *   in just a single cycle with an ADD instruction that reads the PC
 *   directly.  But unfortunately the offset needed is alignment
 *   dependent (the PC reads as "current instruction address plus four
 *   rounded down to a 4 byte boundary"), and we're inline code and
 *   don't know our alignment.  That too isn't possible in assembly
 *   syntax, but could be done by a post-processor tool, saving two
 *   cycles total.
 *
 * + As mentioned above, not having the PC at the end of the frame is
 *   a hardship for software.  Consider rearranging the format of a
 *   suspended thread's frame for the benefit of arch_switch.  Also
 *   consider moving LR into the second spot so it can share the same
 *   POP instruction.  Convert both incoming and outgoing threads at
 *   exception-return context switch time (which is an much less
 *   common code path).  This could save six cycles.
 *
 * + The RETPSR value in the context switch frame is actually ignored
 *   by swap, and always stored as a constant (just the thumb bit).
 *   If it's only needed by exception return we might be able to flag
 *   threads as "saved by swap, needs PSR added to frame" (or rather
 *   "not saved by exception" I guess).  Saves another cycle.
 *
 * + FPU can be handled in mostly C code by passing SP as an argument
 *   and then inspecting the frame there.
 *
 * + Once we've committed to a custom frame format, the PSPLIM
 *   handling can be done by pushing one word containing the limit
 *   below the main stack and popping it during resume.  Quicker than
 *   dealing with offsets into the thread struct.
 *
 * + Also the alignment issue mentioned above can be inspected the
 *   same way FPU state is, and we can simply copy the whole 8-word
 *   frame to the top of the stack before entering the swap code
 *   (hardware restore can interpret it and doesn't care).
 *
 */

// TODO:
// + port to be a real swap/switch
// + tracing hook
// + FPU support
// + stack alignment
// + PSPLIM support

K_KERNEL_STACK_DEFINE(thread2_stk, 4096);
static struct k_thread thread2, *main_thread;

static ALWAYS_INLINE void my_swap(void *old_cs, void *new_cs)
{
	register void *r0 __asm__("r0") = old_cs;
	register void *r12 __asm__("r12") = new_cs;

	__asm__ volatile(
		/* Save an "exception frame" to the outgoing stack */
		"mov r1, lr;"
		"ldr r2, =1f;"         /* Restore PC */
		"add r2, r2, #1;"      /* (PC needs thumb bit) */
		"mov r3, #0x01000000;" /* RETPSR: only thumb bit needed */
		"push {r0-r3};"        /* Top four words of exc frame */
		"sub sp, sp, #16;"     /* Leave junk in bottom four words */

		/* Write the outgoing callee_saved registers */
		"stmia r0!, {r4-r11};"
		"str sp, [r0];"

		/* Swap stacks and enable interrupts */
		"ldr sp, [r12, #32];"
		"mov r1, #0;"
		"mrs r1, basepri;"

		/*  FIXME: call tracing hook here-ish */

		/* Fetch incoming APSR flags into r2 and move the
		 * PC-to-restore value into the last spot in the frame
		 */
		"ldr r1, [sp, #24];"
		"ldr r2, [sp, #28];"
		"str r1, [sp, #28];"

		/* Load the remaining callee_saved of the incoming thread */
		"ldm r12, {r4-r11};"

		/* Restore flags and remaining GPR state */
		"mrs r2, apsr;"
		"pop {r0-r3};"
		"ldr r12, [sp];"
		"ldr lr, [sp, #4];"
		"add sp, sp, #12;"
		"pop {pc};"
		"1:" /* Label for restored PC */

		: "+r"(r0), "+r"(r12) :: "r1", "r2", "r3");
}

void thread2_fn(void *a, void *b, void *c)
{
	uint32_t *mainsp = (void *)main_thread->callee_saved.psp;

	printk("In thread2, swapping back to PC 0x%x...\n", mainsp[6]);
	my_swap(&thread2.callee_saved, &main_thread->callee_saved);
	printk("STILL IN THREAD2?!\n");
}

int main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD_TARGET);

	main_thread = _current;

	k_thread_create(&thread2, thread2_stk, K_KERNEL_STACK_SIZEOF(thread2_stk),
			thread2_fn, (void*)7, (void*)8, (void*)9,
			0, 0, K_FOREVER);

	printk("In main, swapping...\n");
	my_swap(&main_thread->callee_saved, &thread2.callee_saved);
	printk("Back in main\n");

	return 0;
}
