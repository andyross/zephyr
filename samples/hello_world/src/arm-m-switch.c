/* Copyright 2025 The ChromiumOS Authors
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/sys/util.h>
#include "arm-m-switch.h"

// TODO:
//
// + arch_float_en/disable(), also need to clear FPU flag on switch
//   so it doesn't propagate to non-FPU threads by accident.
// + Cortex M0 (ARMv6) support (some LDM/STM variants aren't there?)
// + CONFIG_DEBUG_THREAD_INFO is tied to the old frame format and some
//   samples turn it on.  Also EXTRA_EXCEPTION_INFO is involved here.
// + Need to track and restore CONTROL.nPRIV bit so we can switch
//   between kernel/user threads.
// + Userspace needs some thought & rewrite, I think.  The SVC arrives
//   on the MSP stack, then needs to "return" into the privileged
//   handler, which then drops privilege and hand-switches back to the
//   hardware frame.  I think?

/* The basic exception frame, popped by the hardware during return */
struct hw_frame_base {
	uint32_t r0, r1, r2, r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t apsr;
};

/* The hardware frame pushed when entry is taken with FPU active */
struct hw_frame_fpu {
	struct hw_frame_base base;
	uint32_t s_regs[16];
	uint32_t fpscr;
	uint32_t reserved;
};

/* The hardware frame pushed when entry happens with a misaligned stack */
struct hw_frame_align {
	struct hw_frame_base base;
	uint32_t align_pad;
};

/* Both of the above */
struct hw_frame_align_fpu {
	struct hw_frame_fpu base;
	uint32_t align_pad;
};

/* Zephyr's synthesized frame used during context switch on interrupt
 * exit.  It's a minimal hardware frame plus storage for r4-11.
 */
struct synth_frame {
	uint32_t r7, r8, r9, r10, r11;
	uint32_t r4, r5, r6;		/* these match switch format */
	struct hw_frame_base base;
};

/* Zephyr's frame used for suspended threads */
struct switch_frame {
#ifdef CONFIG_BUILTIN_STACK_GUARD
	uint32_t psplim;
#endif
	uint32_t apsr;
	uint32_t r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, r10, r11, r12;
	uint32_t lr;
	uint32_t pc;
};

/* Union of synth and switch frame */
union u_frame {
	struct {
		char pad[sizeof(struct switch_frame) - sizeof(struct synth_frame)];
		struct synth_frame hw;
	};
	struct switch_frame sw;
};

/* u_frame with have_fpu flag prepended (zero value), but no FPU data */
struct z_frame {
#ifdef CONFIG_FPU_SHARING
	uint32_t have_fpu;
#endif
	union u_frame u;
};

/* u_frame + FPU data, with have_fpu (non-zero) */
struct z_frame_fpu {
	uint32_t have_fpu;
	uint32_t s_regs[32];
	uint32_t fpscr;
	union u_frame u;
};

/* Union of all possible stack frame formats, aligned at the top (!).
 * Note that FRAMESZ is constructed to be larger than any of them to
 * avoid having a zero-length array.  The code doesn't ever use the
 * size of this struct, it just wants to be have compiler-visible
 * offsets for in-place copies.
 */
#define FRAMESZ (4 + MAX(sizeof(struct z_frame_fpu), sizeof(struct hw_frame_align_fpu)))
#define PAD(T) char pad_##T[FRAMESZ - sizeof(struct T)]
union frame {
	struct { PAD(hw_frame_base);      struct hw_frame_base hw;          };
	struct { PAD(hw_frame_fpu);       struct hw_frame_fpu hwfp;         };
	struct { PAD(hw_frame_align);     struct hw_frame_align hw_a;       };
	struct { PAD(hw_frame_align_fpu); struct hw_frame_align_fpu hwfp_a; };
	struct { PAD(z_frame);            struct z_frame z;                 };
	struct { PAD(z_frame_fpu);        struct z_frame_fpu zfp;           };
};

/* Validate the structs are correctly top-aligned */
#define FRAME_FIELD_END(F) ((void *)&(&(((union frame *)0)->F))[1])
BUILD_ASSERT(FRAME_FIELD_END(hw) == FRAME_FIELD_END(hwfp));
BUILD_ASSERT(FRAME_FIELD_END(hw) == FRAME_FIELD_END(hw_a));
BUILD_ASSERT(FRAME_FIELD_END(hw) == FRAME_FIELD_END(hwfp_a));
BUILD_ASSERT(FRAME_FIELD_END(hw) == FRAME_FIELD_END(z));
BUILD_ASSERT(FRAME_FIELD_END(hw) == FRAME_FIELD_END(zfp));

/* Global pointers to the frame locations for the callee-saved
 * registers.  Set in arm_m_must_switch(), and used by the fixup
 * assembly in arm_m_exc_exit.
 */
struct { void *out, *in; } arm_m_cs_ptrs;

// FIXME: the use of the tmp structs in the copy macros here forces
// the compiler to zero-fill unused fields needlessly.  Should use
// individual variables.

/* Emits an in-place copy from a hw_frame_base to a switch_frame */
#define HW_TO_SWITCH(hw, sw) do {				\
	struct switch_frame swtmp = {				\
		.r0 = hw.r0, .r1 = hw.r1, .r2 = hw.r2,		\
		.r3 = hw.r3, .r12 = hw.r12, .lr = hw.lr,	\
		.pc = hw.pc, .apsr = hw.apsr,			\
	};							\
	swtmp.pc |= 1; /* thumb bit! */				\
	sw = swtmp;						\
} while(false)

/* Emits an in-place copy from a switch_frame to a synth_frame */
#define SWITCH_TO_SYNTH(sw, syn) do {					\
	struct synth_frame syntmp = {					\
		.r4 = sw.r4, .r5 = sw.r5, .r6 = sw.r6, .r7 = sw.r7,	\
		.r8 = sw.r8, .r9 = sw.r9, .r10 = sw.r10, .r11 = sw.r11,	\
		.base.r0 = sw.r0, .base.r1 = sw.r1, .base.r2 = sw.r2,	\
		.base.r3 = sw.r3, .base.r12 = sw.r12, .base.lr = sw.lr,	\
		.base.pc = sw.pc, .base.apsr = sw.apsr,			\
	};								\
	syn = syntmp;							\
} while(false)

/* Reports if the passed return address is a valid EXC_RETURN (high
 * four bits set) that will restore to the PSP running in thread mode
 * (low four bits == 0xd).  That is an interrupted Zephyr thread
 * context.  For everything else, we just return directly via the
 * hardware-pushed stack frame with no special handling. See ARMv7M
 * manual B1.5.8.
 */
static bool arm_m_is_thread_return(uint32_t lr)
{
	return (lr & 0xf000000f) == 0xf000000d;
}

/* Returns true if the EXC_RETURN address indicates a FPU subframe was
 * pushed to the stack.  See ARMv6M manual B1.5.8.
 */
static bool arm_m_fpu_state_pushed(uint32_t lr)
{
	return IS_ENABLED(CONFIG_CPU_HAS_FPU) ? !!(lr & 0x08000000) : false;
}

#ifdef CONFIG_BUILTIN_STACK_GUARD
#define PSPLIM(f) ((f)->z.u.sw.psplim)
#else
#define PSPLIM(f) 0
#endif

/* Converts, in place, a pickled "switch" frame from a suspended
 * thread to a "synthesized" format that can be restored by the CPU
 * hardware on exception exit.
 */
static void *arm_m_switch_to_cpu(void *sp)
{
	union frame *f;
	uint32_t splim;

#ifdef CONFIG_FPU_SHARING
	bool have_fpu = !!*(uint32_t *)sp;

	if (have_fpu) {
		f = CONTAINER_OF(sp, union frame, zfp.have_fpu);
		splim = PSPLIM(f);
		__asm__ volatile("vldm %0, {s0-s31}" :: "r"(&f->zfp.s_regs[0]));
		SWITCH_TO_SYNTH(f->zfp.u.sw, f->zfp.u.hw);
	} else {
		f = CONTAINER_OF(sp, union frame, z.have_fpu);
		splim = PSPLIM(f);
		SWITCH_TO_SYNTH(f->z.u.sw, f->zfp.u.hw);
	}
#else
	f = CONTAINER_OF(sp, union frame, z.u.sw);
	splim = PSPLIM(f);
	SWITCH_TO_SYNTH(f->z.u.sw, f->z.u.hw);
#endif

#ifdef CONFIG_BUILTIN_STACK_GUARD
	__asm__ volatile("msr psplim, %0" :: "r"(splim));
#endif

	/* Mark the callee-saved pointer for the fixup assembly.  Note
         * funny layout that puts r7 first!
         */
        arm_m_cs_ptrs.in = &f->z.u.hw.r7;

	return &f->z.u.hw.base;
}

static void fpu_cs_copy(struct hw_frame_fpu *src, struct z_frame_fpu *dst)
{
	for (int i = 0; IS_ENABLED(CONFIG_FPU_SHARING) && i < 16; i++) {
		dst->s_regs[i] = src->s_regs[i];
	}
}

/* Converts, in-place, a CPU-spilled ("hardware") exception entry
 * frame to our ("zephyr") switch handle format such that the thread
 * can be suspended
 */
static void *arm_m_cpu_to_switch(void *sp, bool fpu)
{
	union frame *f;
	struct hw_frame_base *base = sp;
	bool padded = (base->apsr & 0x200);
	uint32_t fpscr;

	/* NOTE: the converted switch frame can be bigger than the
	 * input hardware frame.  We should be checking stack bounds
	 * here when PSPLIM is enabled.
	 */

	if (fpu && IS_ENABLED(CONFIG_FPU_SHARING)) {
		uint32_t dummy;

		/* Lazy FPU stacking is enabled, so before we touch
		 * the stack frame we have to tickle the FPU to force
		 * it to spill the caller-save registers.  Then clear
		 * CONTROL.FPCA which gets set again by that instruction.
		 */
		__asm__ volatile("vmov %0, s0;"
				 "mrs %0, control;"
				 "bic %0, %0, #4;"
				 "msr control, %0;"
				 :: "r"(dummy));
	}

	if (IS_ENABLED(CONFIG_FPU_SHARING) && fpu) {
		fpscr = CONTAINER_OF(sp, struct hw_frame_fpu, base)->fpscr;
	}

	/* There are four (!) different offsets from the interrupted
	 * stack at which the hardware frame might be found at
	 * runtime.  These expansions let the compiler generate
	 * optimized in-place copies for each.  In practice it does a
	 * pretty good job, much better than a double-copy via an
	 * intermediate buffer.  Note that when FPU state is spilled
	 * we must copy the 16 spilled registers first, to make room
	 * for the copy.
	 */
	// FIXME: switch frame has invariant location at f->z.u.sw,
	// shouldn't be a macro agument
	if (!fpu && !padded) {
		f = CONTAINER_OF(sp, union frame, hw.r0);
		HW_TO_SWITCH(f->hw, f->z.u.sw);
	} else if (!fpu && padded) {
		f = CONTAINER_OF(sp, union frame, hw_a.base.r0);
		HW_TO_SWITCH(f->hw_a.base, f->z.u.sw);
	} else if (fpu && !padded) {
		f = CONTAINER_OF(sp, union frame, hwfp.base.r0);
		fpu_cs_copy(&f->hwfp, &f->zfp);
		HW_TO_SWITCH(f->hwfp.base, f->z.u.sw);
	} else if (fpu && padded) {
		f = CONTAINER_OF(sp, union frame, hwfp_a.base.base.r0);
		fpu_cs_copy(&f->hwfp_a.base, &f->zfp);
		HW_TO_SWITCH(f->hwfp_a.base.base, f->z.u.sw);
	}

#ifdef CONFIG_BUILTIN_STACK_GUARD
	__asm__ volatile("mrs %0, psplim" : "=r"(f->z.u.sw.psplim));
#endif

        /* Mark the callee-saved pointer for the fixup assembly */
        arm_m_cs_ptrs.out = &f->z.u.sw.r4;

#ifdef CONFIG_FPU_SHARING
	if (fpu) {
		__asm__ volatile("vstm %0, {s16-s31}" :: "r"(&f->zfp.s_regs[16]));
		f->zfp.fpscr = fpscr;
		f->zfp.have_fpu = true;
		return &f->zfp.have_fpu;
	} else {
		f->z.have_fpu = false;
		return &f->z.have_fpu;
	}
#endif
	return &f->z.u.sw;
}

/* Constructs a new stack in the provided region (aligned to and in
 * units of 8 bytes per AAPCS) and returns the switch handle
 */
void *arm_m_new_stack(char *base, uint32_t sz, void *entry,
		      void *arg0, void *arg1, void *arg2)
{
	struct switch_frame *sw;
	uint32_t baddr;

	baddr = ((uint32_t)base + 7) & ~7;
	sz = ((uint32_t)(base + sz) - baddr) & ~7;

	if (sz < sizeof(struct switch_frame)) {
		return NULL;
	}

	/* Note: a useful trick here would be to initialize LR to
	 * point to cleanup code, avoiding the need for the
	 * z_thread_entry wrapper, saving a few words of stack frame
	 * and a few cycles on thread entry.
	 */
	sw = (void *)(baddr + sz - sizeof(*sw));
	*sw = (struct switch_frame) {
		IF_ENABLED(CONFIG_BUILTIN_STACK_GUARD, (.psplim = baddr,))
		.r0 = (uint32_t) arg0,
		.r1 = (uint32_t) arg1,
		.r2 = (uint32_t) arg2,
		.pc = ((uint32_t) entry) | 1, /* set thumb bit! */
		.apsr = 0x1000000,            /* thumb bit here too! */
	};

	if (IS_ENABLED(CONFIG_FPU_SHARING)) {
		return CONTAINER_OF(sw, struct z_frame, u.sw);
	}
	return sw;
}

void *arm_m_last_switch_handle;

void *z_get_next_switch_handle(void *interrupted);

bool arm_m_must_switch(uint32_t lr)
{
	if (!arm_m_is_thread_return(lr)) {
		return false;
	}

	void *last, *next = z_get_next_switch_handle(NULL);

	if (next == NULL) {
		return false;
	}

	bool fpu = arm_m_fpu_state_pushed(lr);

	/* Rejigger the frame we're pickling, and unpickle the new
	 * thread we're returning into
	 */
	__asm__ volatile("mrs %0, psp" : "=r"(last));
	last = arm_m_cpu_to_switch(last, fpu);
	next = arm_m_switch_to_cpu(next);
	__asm__ volatile("msr psp, %0" :: "r"(next));

#if !defined(CONFIG_MULTITHREADING)
	arm_m_last_switch_handle = last;
#elif defined(CONFIG_USE_SWITCH)
	arch_current_thread()->base.switch_handle = last;
#endif

	return true;
}

/* We arrive here on "return" from exception handlers on a context
 * switch. Our job is to save the interrupted r4-r11 of the outgoing
 * thread (which can't be done in the C function context because we
 * can't control the compiler spill behavior of callee-saved
 * registers), restore the same registers from the incoming thread,
 * and then return via BX to a constructed exception return value.  We
 * can use any of r0-r3/r12/lr because they have already been saved.
 * FPU restore is handled in software, so we always use a constant
 * EXC_RETURN value indicating an integer-only restore.
 */
__asm__(".globl arm_m_exc_exit;"
	"arm_m_exc_exit:;"
	"  ldr r0, =arm_m_cs_ptrs;"
	"  ldm r0, {r0, r1};" /* fields: out, in */
	"  mov lr, #0xfffffffd;"
	"  stm r0, {r4-r11};"
	"  ldm r1!, {r7-r11};"
	"  ldm r1, {r4-r6};"
	"  bx lr;");
