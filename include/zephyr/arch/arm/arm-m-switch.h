/* Copyright 2025 The ChromiumOS Authors
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _ZEPHYR_ARCH_ARM_M_SWITCH_H
#define _ZEPHYR_ARCH_ARM_M_SWITCH_H

/* Need this to break a header cycle vs. zephyr/arch/arm/arch.h */
#define ARCH_STACK_PTR_ALIGN 8

#include <stdint.h>
#include <zephyr/kernel_structs.h>
#include <zephyr/kernel/thread.h>
#include <zephyr/kernel/thread_stack.h>

void *arm_m_new_stack(char *base, uint32_t sz, void *entry,
		      void *arg0, void *arg1, void *arg2, void *arg3);

bool arm_m_must_switch(uint32_t lr);

void arm_m_exc_exit(void);

/* Local declarations for symbols that lack headers or which can't be
 * included here for header dependency reasons.
 */
//extern char z_interrupt_stacks[1][CONFIG_ISR_STACK_SIZE];
void z_arm_configure_dynamic_mpu_regions(struct k_thread *thread);
extern uintptr_t z_arm_tls_ptr;

K_KERNEL_STACK_ARRAY_DECLARE(z_interrupt_stacks, CONFIG_MP_MAX_NUM_CPUS, CONFIG_ISR_STACK_SIZE);

static inline void arm_m_exc_tail(void)
{
	if (!IS_ENABLED(CONFIG_MULTITHREADING)) {
		return;
	}

	/* Dirty trickery: we load this ISR's LR register (which
	 * contains our interrupt return token) from the runtime stack
	 * frame pushed to the top of the interrupt stack on entry.
	 * Check it to see if we can/should return to a different
	 * thread (which will then have magically pickled our
	 * interrupted stack into "switch" format), and then if so:
	 * CLOBBER it with the address of our fixup code so that we
	 * can finish saving the interrupted r4-r11 registers before
	 * returning from the interrupt.
	 *
	 * Obviously this only works if the ISR is "ABI-compliant
	 * enough".  It doesn't have to have pushed a complete frame,
	 * but it does have to have put LR into its standard location.
	 * In practice generated code does (because it has to store LR
	 * somewhere so it can call other functions and then pop it to
	 * return), so this works even on code built with
	 * -fomit-frame-pointer.  If an app needs a direct interrupt
	 * and can't meet these requirents, it can always skip this
	 * call and return directly (reschedule is optional for direct
	 * interrupts anyway).
	 */
	char *stack = (char *)K_KERNEL_STACK_BUFFER(z_interrupt_stacks[0]);
	uint32_t *s_top = (uint32_t *)(stack +
				       K_KERNEL_STACK_SIZEOF(z_interrupt_stacks[0]));
	uint32_t *lr_ptr = &s_top[-1];

	if (arm_m_must_switch(*lr_ptr)) {
		*lr_ptr = 1 | (uint32_t)arm_m_exc_exit; /* thumb bit! */
	}
}

static ALWAYS_INLINE void arm_m_switch(void *switch_to, void **switched_from)
{
#if defined(CONFIG_USERSPACE) || defined(CONFIG_MPU_STACK_GUARD)
	z_arm_configure_dynamic_mpu_regions(_current);
#endif
#ifdef CONFIG_THREAD_LOCAL_STORAGE
	z_arm_tls_ptr = _current->tls;
#endif

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
		 "sub sp, sp, #24;"       /* skip over space for r6-r11 */
		 "push {r0-r5};"
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
		 /* Push FPU state (if active) to our outgoing stack */
		 "   mrs r8, control;"    /* read CONTROL.FPCA */
		 "   and r7, r8, #4;"     /* r7 == have_fpu */
		 "   cbz r7, 1f;"
		 "   bic r8, r8, #4;"     /* clear CONTROL.FPCA */
		 "   msr control, r8;"
		 "   vmrs r6, fpscr;"
		 "   push {r6};"
		 "   vpush {s0-s31};"
		 "1: push {r7};"          /* have_fpu word */

		 /* Pop FPU state (if present) from incoming frame in r4 */
		 "   ldm r4!, {r7};"      /* have_fpu word */
		 "   cbz r7, 2f;"
		 "   vldm r4!, {s0-s31};" /* (note: sets FPCA bit for us) */
		 "   ldm r4!, {r6};"
		 "   vmsr fpscr, r6;"
		 "2:;"
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

		 "3:" /* Label for restore address */
		 :: "r"(r4), "r"(r5) :
		  "r6", "r7", "r8", "r9", "r10", "r11");
}

#ifdef CONFIG_USE_SWITCH
static ALWAYS_INLINE void arch_switch(void *switch_to, void **switched_from)
{
	arm_m_switch(switch_to, switched_from);
}
#endif

#endif /* _ZEPHYR_ARCH_ARM_M_SWITCH_H */
