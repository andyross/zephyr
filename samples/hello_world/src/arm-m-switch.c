#include <zephyr/sys/util.h>
#include <zephyr/kernel/thread.h>
//#include <ksched.h> //DEBUG

// Testing: mps3/corstone300/an547 is a qemu platform with both FPU and PSPLIM

void *z_get_next_switch_handle(void *interrupted);//DEBUG, from ksched.h


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
		__asm__ volatile("mov lr, =arm_m_exc_exit");
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
		 "2: ;"
		 "   msr control, r7;"
#endif

		 /* Save the outgoing switch handle (which is SP), swap stacks,
		  * and enable interrupts.  The restore process is
		  * interruptible code (running in the incoming thread) once
		  * the stack is valid.
		  */
		 "str sp, [r5];"
		 "mov r4, sp;"
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

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

/* The basic exception frame, popped by the hardware during return */
struct hw_frame_base {
	uint32_t r0, r1, r2, r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t apsr;
};

/* The hardware frame when entry is taken with FPU active */
struct hw_frame_fpu {
	struct hw_frame_base base;
	uint32_t s_regs[16];
	uint32_t fpscr;
	uint32_t reserved;
};

/* The hardware frame when entry happens with a misaligned stack */
struct hw_frame_align {
	struct hw_frame_base base;
	uint32_t align_pad;
};

/* Both of the above */
struct hw_frame_align_fpu {
	struct hw_frame_fpu base;
	uint32_t align_pad;
};

/* Zephyr's synthesized frame used during context switch */
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

union u_frame {
	struct {
		char pad[sizeof(struct switch_frame) - sizeof(struct synth_frame)];
		struct synth_frame hw;
	};
	struct switch_frame sw;
};

struct z_frame {
#ifdef CONFIG_FPU_SHARING
	uint32_t have_fpu;
#endif
	union u_frame u;
};

struct z_frame_fpu {
	uint32_t have_fpu;
	uint32_t s_regs[32];
	uint32_t fpscr;
	union u_frame u;
};

#define FRAMESZ (4 + MAX(sizeof(struct z_frame_fpu), sizeof(struct hw_frame_align_fpu)))

/* Union of all possible stack frame formats, aligned at the top (!).
 * Note that FRAMESZ is constructed to be larger than any of them to
 * avoid having a zero-length array.  The code doesn't ever use the
 * size of this struct, it just wants to be have compiler-visible
 * offsets for in-place copies.
 */
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
#ifdef CONFIG_FPU_SHARING
BUILD_ASSERT(FRAME_FIELD_END(hw) == FRAME_FIELD_END(zfp));
#endif

/* Pointers to the frame locations for the callee-saved registers, set
 * in arm_m_must_switch() and used by the fixup assembly in
 * arm_m_exc_exit.  Also a constant EXC_RETURN as thumb can't have
 * immediates that big.
 */
static uint32_t *cs_outgoing;
static uint32_t *cs_incoming;
static const uint32_t exc_ret = 0xf000000f;

/* Emits an in-place copy from a hw_frame_base to a switch_frame */
#define HW_TO_SWITCH(hw, sw) do {				\
	struct switch_frame swtmp = {				\
		.r0 = hw.r0, .r1 = hw.r1, .r2 = hw.r2,		\
		.r3 = hw.r3, .r12 = hw.r12, .lr = hw.lr,	\
		.pc = hw.pc, .apsr = hw.apsr,			\
	};							\
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
 * (low four bits).  That is an interrupted Zephyr thread context.
 * For everything else, we just return directly via the
 * hardware-pushed stack frame with no special handling. See ARMv7M
 * manual B1.5.8.
 */
static bool arm_m_is_thread_return(uint32_t lr)
{
	return (lr & 0xf000000f) == 0xf000000f;
}

/* Returns true if the EXC_RETURN address indicates a FPU subframe was
 * pushed to the stack.  See ARMv6M manual B1.5.8.
 */
static bool arm_m_fpu_state_pushed(uint32_t lr)
{
	return IS_ENABLED(CONFIG_CPU_HAS_FPU) ? !!(lr & 0x08000000) : false;
}

/* Converts, in place, a pickled "switch" frame from a suspended
 * thread to a "synthesized" format that can be restored by the CPU
 * hardware on exception exit.
 */
static void arm_m_switch_to_cpu(void *sp)
{
	union frame *f;

#ifdef CONFIG_FPU_SHARING
	bool have_fpu = !!*(uint32_t *)sp;

	if (have_fpu) {
		f = CONTAINER_OF(sp, union frame, zfp.have_fpu);
		SWITCH_TO_SYNTH(f->zfp.u.sw, f->zfp.u.hw);
		__asm__ volatile("vldm %0, {r0-s31}" :: "r"(&f->zfp.s_regs[0]));
	} else {
		f = CONTAINER_OF(sp, union frame, z.have_fpu);
		SWITCH_TO_SYNTH(f->z.u.sw, f->zfp.u.hw);
	}
#else
	f = CONTAINER_OF(sp, union frame, z.u.sw);
	SWITCH_TO_SYNTH(f->z.u.sw, f->z.u.hw);
#endif

	/* Mark the callee-saved pointer for the fixup assembly.  Note
         * funny layout that puts r7 first!
         */
        cs_incoming = &f->z.u.hw.r7;

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

        /* Mark the callee-saved pointer for the fixup assembly */
        cs_outgoing = &f->z.u.sw.r4;

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

	sw = (void *)(baddr + sz - sizeof(*sw));
	*sw = (struct switch_frame) {
		IF_ENABLED(CONFIG_BUILTIN_STACK_GUARD, (.psplim = baddr,))
		.r0 = (uint32_t) arg0,
		.r1 = (uint32_t) arg1,
		.r2 = (uint32_t) arg2,
		.pc = ((uint32_t) entry) | 1, /* set thumb bit! */
	};
	return sw;
}

bool arm_m_must_switch(uint32_t lr)
{
	if (!arm_m_is_thread_return(lr)) {
		return false;
	}

	void *last, *next = z_get_next_switch_handle(NULL);

	if (next == NULL) {
		return false;
	}

	__asm__ volatile("mrs %0, psp" : "=r"(last));

	bool fpu = arm_m_fpu_state_pushed(lr);

	/* Rejigger the frame we're pickling, and unpickle the new
	 * thread we're returning into
	 */
	last = arm_m_cpu_to_switch(last, fpu);
	arm_m_switch_to_cpu(next);

	// FIXME: switch_handle disabled until final wiring
	//arch_current_thread()->base.switch_handle = last;

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
__asm__("arm_m_exc_exit:;"
	"  ldr r0, =cs_outgoing;"
	"  ldr r1, =cs_incoming;"
	"  ldr lr, =exc_ret;" /* 0xf000000f, but can't encode that as immeidate */
	"  stm r0, {r4-r11};"
	"  ldmia r1, {r7-r11};"
	"  ldm r1, {r4-r6};"
	"  bx lr;");
