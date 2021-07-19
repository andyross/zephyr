/*
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <init.h>
#include <kernel.h>
#include <kernel_structs.h>
#include <toolchain.h>
#include <sys/__assert.h>
#include <sys/sys_io.h>

#include <xtensa/config/core-isa.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(soc_mp, CONFIG_SOC_LOG_LEVEL);

#include <soc.h>
#include <arch/xtensa/cache.h>
#include <adsp/io.h>

#include <soc/shim.h>

#include <drivers/ipm.h>
#include <ipm/ipm_cavs_idc.h>

#if CONFIG_MP_NUM_CPUS > 1 && !defined(CONFIG_IPM_CAVS_IDC) && defined(CONFIG_SMP)
#error Need to enable the IPM driver for multiprocessing
#endif

/* ROM wake version parsed by ROM during core wake up. */
#define IDC_ROM_WAKE_VERSION	0x2

/* IDC message type. */
#define IDC_TYPE_SHIFT		24
#define IDC_TYPE_MASK		0x7f
#define IDC_TYPE(x)		(((x) & IDC_TYPE_MASK) << IDC_TYPE_SHIFT)

/* IDC message header. */
#define IDC_HEADER_MASK		0xffffff
#define IDC_HEADER(x)		((x) & IDC_HEADER_MASK)

/* IDC message extension. */
#define IDC_EXTENSION_MASK	0x3fffffff
#define IDC_EXTENSION(x)	((x) & IDC_EXTENSION_MASK)

/* IDC power up message. */
#define IDC_MSG_POWER_UP	\
	(IDC_TYPE(0x1) | IDC_HEADER(IDC_ROM_WAKE_VERSION))

#define IDC_MSG_POWER_UP_EXT(x)	IDC_EXTENSION((x) >> 2)

const int CPUMASK = (1 << (CONFIG_MP_NUM_CPUS)) - 1;

#ifdef CONFIG_IPM_CAVS_IDC
static const struct device *idc;
#endif

struct cpustart_rec {
	uint32_t		cpu;

	arch_cpustart_t	fn;
	void		*arg;
	uint32_t		vecbase;

	uint32_t		alive;
};

char *z_mp_stack_top;

#ifdef CONFIG_KERNEL_COHERENCE
/* Coherence guarantees that normal .data will be coherent and that it
 * won't overlap any cached memory.
 */
static struct {
	struct cpustart_rec cpustart;
} cpustart_mem;
#else
/* If .data RAM is by default incoherent, then the start record goes
 * into its own dedicated cache line(s)
 */
static __aligned(XCHAL_DCACHE_LINESIZE) union {
	struct cpustart_rec cpustart;
	char pad[XCHAL_DCACHE_LINESIZE];
} cpustart_mem;
#endif

#define start_rec \
	(*((volatile struct cpustart_rec *) \
	   z_soc_uncached_ptr(&cpustart_mem.cpustart)))

static uint32_t cpu_mask;

/* Tiny assembly stub for calling z_mp_entry() on the auxiliary CPUs.
 * Mask interrupts, clear the register window state and set the stack
 * pointer.  This represents the minimum work required to run C code
 * safely.
 *
 * Note that alignment is absolutely required: the IDC protocol passes
 * only the upper 30 bits of the address to the second CPU.
 */
void z_soc_mp_asm_entry(void);
__asm__(".align 4                   \n\t"
	".global z_soc_mp_asm_entry \n\t"
	"z_soc_mp_asm_entry:        \n\t"
	"  rsil  a0, 5              \n\t" /* 5 == XCHAL_EXCM_LEVEL */
	"  movi  a0, 0              \n\t"
	"  wsr   a0, WINDOWBASE     \n\t"
	"  movi  a0, 1              \n\t"
	"  wsr   a0, WINDOWSTART    \n\t"
	"  rsync                    \n\t"
	"  movi  a1, z_mp_stack_top \n\t"
	"  l32i  a1, a1, 0          \n\t"
	"  call4 z_mp_entry         \n\t");
BUILD_ASSERT(XCHAL_EXCM_LEVEL == 5);

int cavs_idc_smp_init(const struct device *dev);

static int ANDY_MP_ENTRY_ENTERED;

void z_mp_entry(void)
{
	volatile int ie;
	uint32_t idc_reg;

	ANDY_MP_ENTRY_ENTERED = 1;

	printk("MP ENTRY!\n");

	/* We don't know what the boot ROM might have touched and we
	 * don't care.  Make sure it's not in our local cache to be
	 * flushed accidentally later.
	 *
	 * Note that technically this is dropping our own (cached)
	 * stack memory, which we don't have a guarantee the compiler
	 * isn't using yet.  Manual inspection of generated code says
	 * we're safe, but really we need a better solution here.
	 */
	z_xtensa_cache_flush_inv_all();

	/* Copy over VECBASE from the main CPU for an initial value
	 * (will need to revisit this if we ever allow a user API to
	 * change interrupt vectors at runtime).
	 */
	ie = 0;
	__asm__ volatile("wsr.INTENABLE %0" : : "r"(ie));
	__asm__ volatile("wsr.VECBASE %0" : : "r"(start_rec.vecbase));
	__asm__ volatile("rsync");

	/* Set up the CPU pointer. */
	_cpu_t *cpu = &_kernel.cpus[start_rec.cpu];

	__asm__ volatile(
		"wsr." CONFIG_XTENSA_KERNEL_CPU_PTR_SR " %0" : : "r"(cpu));

	/* Clear busy bit set by power up message */
	idc_reg = idc_read(IPC_IDCTFC(0), start_rec.cpu) | IPC_IDCTFC_BUSY;
	idc_write(IPC_IDCTFC(0), start_rec.cpu, idc_reg);

#ifdef CONFIG_IPM_CAVS_IDC
	/* Interrupt must be enabled while running on current core */
	irq_enable(DT_IRQN(DT_INST(0, intel_cavs_idc)));
#endif /* CONFIG_IPM_CAVS_IDC */

#ifdef CONFIG_SMP_BOOT_DELAY
	cavs_idc_smp_init(NULL);
#endif

	start_rec.alive = 1;

	start_rec.fn(start_rec.arg);

#if CONFIG_MP_NUM_CPUS == 1
	/* CPU#1 can be under manual control running custom functions
	 * instead of participating in general thread execution.
	 * Put the CPU into idle after those functions return
	 * so this won't return.
	 */
	for (;;) {
		k_cpu_idle();
	}
#endif
}

bool arch_cpu_active(int cpu_num)
{
	return !!(cpu_mask & BIT(cpu_num));
}

static bool soc_mp_initialized = 0;

/* Most confusing register interface ever (seriously: why can't we
 * have a one-way "signal this interrupt on this cpu" register like
 * everyone else?).  (I)ntra (D)SP (C)ommunication is implemented as
 * an array of 128-byte records for use by each core (i.e. so CPU0
 * uses the first set, but has visibility to all).  Each has a set of
 * registers for communication with each other core in the system
 * (including the current core), so 16 sets.  Each set has a register
 * that "initiates" the interrupt with some data from the current core
 * (the outer/bigger index) and those values get shadowed in the
 * "target" registers for the destination core.
 *
 * Note that there is an "extension" register/shadow pair that works
 * the same way except it (1) doesn't trigger an interrupt when
 * written and (2) only has 30 useable bits.  It is in all other ways
 * just scratch storage, the hardware doesn't inspect it.
 *
 * Also note that the top two bits of the extension word are reserved
 * and get dropped on write, that the top bit of the ITC register acts
 * as the interrupt latch.  It *MUST* be set for the message to send,
 * and that the same bit must then be cleared by the target in the
 * corresponding TFC register before another message will work.  (Also
 * that the clearing of the bit at the destination can itself be an
 * interrupt source for the sender, which is cute but weird).
 *
 * So you can send a synchronous message from core "src" (where src is
 * the PRID of the CPU, equal to arch_curr_cpu() in Zephyr) to core
 * "dst" with:
 *
 *     idc[src].core[dst].ietc = BIT(31) | extra_30_bits; // optional
 *     idc[src].core[dst].itc = BIT(31) | message;
 *     while (idc[src].core[dst].itc & BIT(31)) {}
 *
 * And the other side (on cpu "dst", generally in the IDC interrupt
 * handler) will read those same values via:
 *
 *     uint32_t my_msg = idc[dst].core[src].tfc & 0x7fffffff;
 *     uint32_t my_ext = idc[dst].core[src].tefc & 0x3fffffff;
 *     idc[dst].core[src].tfc = 0; // clear high bit to signal completion
 */
struct cavs_idc {
	struct {
		uint32_t tfc;  /* (T)arget (F)rom (C)ore  */
		uint32_t tefc; /*  ^^ + (E)xtension       */
		uint32_t itc;  /* (I)nitiator (T)o (C)ore */
		uint32_t ietc; /*  ^^ + (E)xtension       */
	} core[5];
	uint32_t ctl;          /* control flags */
	uint32_t unused1[11];
};
static volatile struct cavs_idc *idcregs = (void *)0x1200;

#define IDC_BIT 0x100

/* cAVS interrupt mask bits.  Each core has one of these structs
 * indexed in the intctrl[] array.  Each external interrupt source
 * indexes one bit in one of the state structs (one each for Xtensa
 * level 2-5 interrupts).  Write to "set" to set the mask bit to 1 and
 * disable interrupts.  Write a 1 bit to "clear" to force the mask bit
 * to 0 and enable them.
 *
 * See cAVS register documentation for the specific interrupt
 * mappings.
 */
struct cavs_intctrl {
	struct {
		uint32_t set, clear, mask, status;
	} l2, l3, l4, l5;
};
static volatile struct cavs_intctrl *intctrl = (void *)0x78800;

static void dsp_clock_enable(void)
{
	volatile struct soc_dsp_shim_regs *shim = (void *)SOC_DSP_SHIM_REG_BASE;

	/* CLKCTL must enable and poll for the initialization of the
	 * oscillator before touching other bits
	 */
	shim->clkctl |= BIT(29); /* Request LP Ring Oscillator */
	while ((shim->clksts & BIT(29)) == 0) {
	}

	shim->clkctl |= ((CPUMASK << 16) // Disable clock gating for all cores
			 | BIT(2)        // Oscillator CLock Select == HP ring oscillator
			 | BIT(1));      // LMCS == div/4

	/* OK, this is weird.  It's actually the write to CLKCTL that
	 * seems to uncork the CPUs.  After that, they come up in
	 * PWRSTS reliably no matter what value I set for gate control
	 * in PWRCTL...  AND the set of CPUs that come up seems to not
	 * be the ones set in PWRCTL just now but instead the ones
	 * from a PREVIOUS run!?
	 */
	printk("Spin for CLKCTL to launch cores in PWRSTS\n");
	while ((shim->pwrsts & 0xf) != CPUMASK);
}

/* Tiny trampoline stub for MP startup on Tiger Lake.  In that
 * configuration, the other cores don't have their own ROM (which
 * would otherwise wait for an entry point via an IDC interrupt) and
 * begin at reset running at a fixed address at the bottom of LP-SRAM.
 * This code gets copied there, populated with a runtime immediate
 * address in the second word, and then jumps to the real entry point.
 *
 * Mildly complicated by the fact that LP-SRAM is more than an 18-bit
 * immediate jump offset from our entry point in HP-SRAM, that Xtensa
 * immediate values can only be loaded from addresses behind the PC,
 * and that the assembler refuses to emit a L32R instruction that
 * doesn't reference assembler-generated immediate sections (so that
 * one is hand-encoded).
 */
extern uint32_t z_soc_tgl_tramp;
extern uint32_t z_soc_tgl_tramp_end;
__asm__(".align 4                    \n\t"
	".global z_soc_tgl_tramp     \n\t"
	"z_soc_tgl_tramp:            \n\t"
	"  j tramp_after_imm         \n\t" /* 3-byte jump over immediate */
	".align 4                    \n\t"
	"tramp_entry_addr:           \n\t"
	"  .word 0                   \n\t" /* immediate address goes here */
	"tramp_after_imm:            \n\t"
	"  .byte 1, 0xff, 0xff       \n\t" /* l32r a0, tramp_entry_addr */
	"  movi a2, 0x15             \n\t"
	"  wsr.ATOMCTL a2            \n\t" // FIXME: need this?
	"  jx a0                     \n\t" /* jump to the address in a0 */
	".align 4                    \n\t"
	".global z_soc_tgl_tramp_end \n\t"
	"z_soc_tgl_tramp_end:        \n\t"
	);

static void setup_tgl_trampoline(void)
{
        uint32_t *dst = z_soc_uncached_ptr((void *) LP_SRAM_BASE);
	uint32_t *src = &z_soc_tgl_tramp;


	printk("ANDY: setup trampoline at %p\n", dst);

	/* We might have written to it */
	__asm__ volatile("dhi %0, 0" :: "r"(z_soc_cached_ptr(dst)));

	for (int i = 0; &src[i] < &z_soc_tgl_tramp_end; i++) {
		dst[i] = src[i];
	}

	/* Populate the immediate field with the right entry point */
	dst[1] = (uint32_t) z_soc_mp_asm_entry;

	/* This core surely never loaded code from those addresses,
	 * but just to be safe...
	 */
	__asm__ volatile("ihi %0, 0" :: "r"(z_soc_uncached_ptr(dst)));

	// NOTE: can validate this code works by jumping to it and
	// seeing that we end up in the SMP entry path for a second
	// CPU (and then hang, of course)
	//
	//__asm__ volatile("jx %0" :: "r"(dst)); //DEBUG

	printk("TGL LP-SRAM trampoline initialized\n");
}

static void soc_mp_initialize(void)
{
	volatile struct soc_dsp_shim_regs *shim = (void *)SOC_DSP_SHIM_REG_BASE;

	soc_mp_initialized = 1;

	printk("soc_mp_initialize() CPUMASK = 0x%x\n", CPUMASK);
	setup_tgl_trampoline();

	dsp_clock_enable();

	/* Unmask our own IDC interrupt */
	intctrl[0].l2.clear = IDC_BIT;

	/* Turn off BID and BATTR[0] bits in LPSCTL and set FSDPRUN
	 * ("force DSP running").  SOF has code to do this, but only
	 * on a code path that I don't think can ever be hit (enabling
	 * cpu0 from another CPU).  Seems to be a noop.
	 */
	shim->lpsctl = ((shim->lpsctl & ~(BIT(7) | BIT(12))) | BIT(9));

	/* This has to have bottom bits set for any CPUs we want the
	 * host to be able to bring up later.  If power gating isn't
	 * disabled, then the shut themselves back off (at some point,
	 * I can't figure out exactly when) and won't come back up
	 * when flagged in ADSPCS.
	 */
	//shim->pwrctl = CPUMASK;
}

static void cpu_launch(int cpu_num)
{
	volatile struct soc_dsp_shim_regs *shim = (void *)SOC_DSP_SHIM_REG_BASE;

	printk("cpu_launch()\n");

	shim->pwrctl |= BIT(cpu_num) | BIT(4);

	while ((shim->pwrsts & BIT(cpu_num)) == 0) {
	}

	idcregs[0].ctl = BIT(cpu_num); /* so we can signal the CPU */
	idcregs[cpu_num].ctl = BIT(0); /* so it can signal us later */

	intctrl[cpu_num].l2.clear = IDC_BIT;

}

void arch_start_cpu(int cpu_num, k_thread_stack_t *stack, int sz,
		    arch_cpustart_t fn, void *arg)
{
	volatile uint32_t *swregs = z_soc_uncached_ptr((void*)HP_SRAM_WIN0_BASE);
	uint32_t vecbase;

#ifdef CONFIG_IPM_CAVS_IDC
	if (!idc) {
		idc = device_get_binding(DT_LABEL(DT_INST(0, intel_cavs_idc)));
	}
#endif

	/* Setup data to boot core #1 */
	__asm__ volatile("rsr.VECBASE %0\n\t" : "=r"(vecbase));

	start_rec.cpu = cpu_num;
	start_rec.fn = fn;
	start_rec.arg = arg;
	start_rec.vecbase = vecbase;
	start_rec.alive = 0;

	z_mp_stack_top = Z_THREAD_STACK_BUFFER(stack) + sz;

	if (!soc_mp_initialized) {
		soc_mp_initialize();
	}
	printk("Flagging Zephyr alive to host\n");
	swregs[3] = 0x12345600;

	/* Poll on the host to notify us that ADSPCS indicates the
	 * other CPU is running, this has to be done fast (it's a
	 * synchronous IPC interrupt in SOF)
	 */
	printk("Waiting on host to flag CPU ready...\n");
	while ((swregs[3] & BIT(cpu_num)) == 0);

	cpu_launch(cpu_num);

	/* Wait for an in-flight interrupt to clear (FIXME: probably needless) */
	while (idcregs[0].core[cpu_num].itc & 0x80000000) {
	}

	__ASSERT((intctrl[cpu_num].l2.status & IDC_BIT) == 0,
		 "cpu%d IDC interrupt already latched?", cpu_num);

	/* Trigger the IDC interrupt, with the entry point as the payload */
	printk("Sending IDC interrupt...\n");
	idcregs[0].core[cpu_num].ietc = ((long) z_soc_mp_asm_entry) >> 2;
	idcregs[0].core[cpu_num].itc = 0x81000002;

	/* Older API compatibility check */
	__ASSERT(idcregs[0].core[cpu_num].itc == (IDC_MSG_POWER_UP | IPC_IDCITC_BUSY),
		 "msg wrong, expect 0x%x got 0x%x",
		 (int)(IDC_MSG_POWER_UP | IPC_IDCITC_BUSY),
		 idcregs[0].core[cpu_num].itc);
	__ASSERT(idcregs[0].core[cpu_num].ietc
		 == IDC_MSG_POWER_UP_EXT((long)z_soc_mp_asm_entry), "");

	/* Verify that the interrupt controller latched the bit */
	while ((intctrl[cpu_num].l2.status & IDC_BIT) == 0) {
	}
	printk("IDC interrupt latched\n");

	__ASSERT((intctrl[cpu_num].l2.mask & IDC_BIT) == 0,
		 "cpu%d IDC interrupt masked!", cpu_num);

	//printk("HACK: jump straight to trampoline\n");
	//__asm__ volatile("jx %0" :: "r"(LP_SRAM_BASE));

#if 0
	/* Spin, waiting for the interrupt to show handled on the other CPU */
	while ((intctrl[cpu_num].l2.status & IDC_BIT) != 0) {
	}
	printk("IDC Interrupt handled\n");

	/* Spin, waiting for the other side to clear the DONE bit */
	while(idcregs[0].core[cpu_num].itc & BIT(31)) {
	}
#endif
	idcregs[0].core[cpu_num].itc = 0;

#ifdef CONFIG_SMP_BOOT_DELAY
	cavs_idc_smp_init(NULL);
#endif

	printk("Waiting for MP core\n");
	while (!start_rec.alive)
		;

	/*
	 * No locking needed as long as CPUs can only be powered on by the main
	 * CPU and cannot be powered off
	 */
	cpu_mask |= BIT(cpu_num);
}

#ifdef CONFIG_SCHED_IPI_SUPPORTED
FUNC_ALIAS(soc_sched_ipi, arch_sched_ipi, void);
void soc_sched_ipi(void)
{
	if (idc != NULL) {
		ipm_send(idc, 0, IPM_CAVS_IDC_MSG_SCHED_IPI_ID,
			 IPM_CAVS_IDC_MSG_SCHED_IPI_DATA, 0);
	}
}
#endif
