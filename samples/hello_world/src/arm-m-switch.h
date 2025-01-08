#ifndef _ZEPHYR_ARCH_ARM_M_SWITCH_H
#define _ZEPHYR_ARCH_ARM_M_SWITCH_H

#include <zephyr/sys/util.h>
#include <zephyr/kernel/thread.h>

/* Declared extern here to simplify unit testing; normally found in
 * ksched.h, which is a kernel-only header.
 */
void *z_get_next_switch_handle(void *interrupted);

void *arm_m_new_stack(char *base, uint32_t sz, void *entry,
		      void *arg0, void *arg1, void *arg2);

bool arm_m_must_switch(uint32_t lr);

static ALWAYS_INLINE void arm_m_exc_tail(void)
{
	/* This is sort of a dirty trick: we clobber the LR register
	 * deliberately so our caller (note carefully ALWAYS_INLINE!)
	 * "returns" into our fixup assembly and not into a hardware
	 * restore.  Strictly this isn't legal: LR is a general
	 * purpose register owned by the compiler and we don't know
	 * that it hasn't generated code to mutate/restore/reuse it
	 * before return.  In practice it doesn't because that would
	 * break the debugger ABI.  But it remains a dirty trick.
	 */
	uint32_t lr;

	__asm__ volatile("mov %0, lr" : "=r"(lr));
	if (arm_m_must_switch(lr)) {
		__asm__ volatile("ldr lr, =arm_m_exc_exit");
	}
}

static ALWAYS_INLINE void arm_m_switch(void *switch_to, void **switched_from)
{
	/* new switch handle in r4, old switch handle pointer in r5.
	 * r6-r8 are used by the code here, and r9-r11 are
	 * unsaved/clobbered (they are very likely to be caller-saved
	 * registers in the enclosing function that the compiler can
	 * avoid using, i.e. we can let it make the call and avoid a
	 * double-spill).  But all registers are restored fully
	 * (because we might be switching to an interrupt-saved frame)
	 */
	register uint32_t r4 __asm__("r4") = (uint32_t) switch_to;
	register uint32_t r5 __asm__("r5") = (uint32_t) switched_from;
	__asm__ volatile(
		 /* Construct and push a {r12, lr, pc} group at the top
		  * of the frame, where PC points to the final restore location
		  * at the end of this sequence.
		  */
		 "mov r6, r12;"
		 "mov r7, lr;"
		 "ldr r8, =3f;"           /* address of restore PC */
		 "add r8, r8, #1;"        /* set thumb bit */
		 "push {r6-r8};"
		 "sub sp, sp, #32;"       /* skip over space for r5-r11 */
		 "push {r0-r3};"
		 "mov r2, #0x01000000;"   /* APSR (only care about thumb bit) */
		 "mov r0, #0;"            /* Leave r0 zero for code blow */
#ifdef CONFIG_BUILTIN_STACK_GUARD
		 "mrs r1, psplim;"
		 "push {r1-r2};"
		 "msr psplim, r0;"        /* zero it so we can move the stack */
#else
		 "push {r2};"
#endif

#ifdef CONFIG_FPU_SHARING
		 "   mrs r8, control;"   /* read CONTROL.FPCA */
		 "   or r7, r8, ~4;"     /* cleared FPCA in r7 */
		 "   tst r8, 4;"
		 "   beq r8, 1f;"        /* check FPCA */
		 "   mrs r6, fpscr;"     /* save FPU state to stack */
		 "   push {r6};"
		 "   vstmdb sp, s0-s31;"
		 "1: push {r8};"          /* outgoing have_fpu */
		 "   ldmia r4, {r8};"     /* incoming have_fpu */
		 "   cmp r4, #0;"
		 "   beq r8, 2f;"
		 "   vldmia r4, {s0-s31};" /* restore FPU state */
		 "   ldmia r4, {r6};"
		 "   msr FPSCR, r6;"
		 "   or r7, r7, 4;"       /* set FPCA */
		 "2: msr control, r7;"
#endif

		 /* Save the outgoing switch handle (which is SP), swap stacks,
		  * and enable interrupts.  The restore process is
		  * interruptible code (running in the incoming thread) once
		  * the stack is valid.
		  */
		 "str sp, [r5];"
		 "mov sp, r4;"
		 "msr basepri, r0;"

		 /* Restore is super simple: pop the flags (and stack limit if
		  * enabled) then slurp in the whole GPR set in two
		  * instructions. (The instruction encoding disallows popping
		  * both LR and PC in a single instruction)
		  */
#ifdef CONFIG_BUILTIN_STACK_GUARD
		 "pop {r1-r2};"
		 "msr psplim, r1;"
#else
		 "pop {r2};"
#endif
		 "msr apsr_nzcvq, r2;" /* bonkers syntax */
		 "pop {r0-r12, lr};"
		 "pop {pc};"

		 "3:"
		 :: "r"(r4), "r"(r5) :
		  "r6", "r7", "r8", "r9", "r10", "r11");
}

#endif /* _ZEPHYR_ARCH_ARM_M_SWITCH_H */
