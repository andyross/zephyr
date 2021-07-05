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

void z_mp_entry(void)
{
	volatile int ie;
	uint32_t idc_reg;
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
 * and get dropped on write, that the top bit of the ITC register
 * *MUST* be set for the message to send, and that the same bit must
 * then be cleared by the target in the corresponding TFC register
 * before another message will work.  (Also that the clearing of the
 * bit at the destination can itself be an interrupt source for the
 * sender, which is cute but weird).
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

struct cavs_intctrl {
	struct {
		uint32_t set, clear, mask, status;
	} l2, l3, l4, l5;
};
static volatile struct cavs_intctrl *intctrl = (void *)0x78800;

static void soc_mp_initialize(void)
{
	volatile uint32_t *swregs = z_soc_uncached_ptr((void*)HP_SRAM_WIN0_BASE);
	volatile struct soc_dsp_shim_regs *shim = (void *)SOC_DSP_SHIM_REG_BASE;

	soc_mp_initialized = 1;

#ifdef CONFIG_IPM_CAVS_IDC
	// FIXME: remove
	idc = device_get_binding(DT_LABEL(DT_INST(0, intel_cavs_idc)));
#endif

	printk("Enabling IDC interrupts to all cores:\n");
	for (int c = 0; c < CONFIG_MP_NUM_CPUS; c++) {
		printk("  idcregs[%d].ctl : 0x%x -> ", c, idcregs[c].ctl);
		idcregs[c].ctl = 0xff;
		printk("0x%x\n", idcregs[c].ctl);
	}

	printk("Unmasking L2 interrupts on all cores\n");
	for (int c = 0; c < CONFIG_MP_NUM_CPUS; c++) {
		// FIXME: clear just the bit we need
		intctrl[c].l2.clear = 0xffffffff;
	}

	printk("PWRCTL = 0x%x PWRSTS = 0x%x LPSCTL = 0x%x CLKCTL = 0x%x CLKSTS = 0x%x\n",
	       shim->pwrctl, shim->pwrsts, shim->lpsctl, shim->clkctl, shim->clksts);

	printk("Set LPSCTL.FDSPRUN\n");
	shim->lpsctl = BIT(9);

	printk("Disable idle clock gating on all cores\n");
	shim->clkctl |= BIT(29); // DEFAULT_RO/RLROSCC
	while ((shim->clksts & BIT(29)) == 0);

	shim->clkctl |= (0xff << 16); // Disable clock gating for all cores
	shim->clkctl |= BIT(2); // Oscillator CLock Select == HP ring oscillator
	shim->clkctl |= BIT(1); // LMCS == div/4

	/* FIXME: Try turning it off here, then on on start */
	printk("ENabling idle power gating on all cores\n");
	shim->pwrctl &= ~(0xf | BIT(4) | BIT(6));

	printk("PWRCTL = 0x%x PWRSTS = 0x%x LPSCTL = 0x%x CLKCTL = 0x%x CLKSTS = 0x%x\n",
	       shim->pwrctl, shim->pwrsts, shim->lpsctl, shim->clkctl, shim->clksts);

	k_busy_wait(500000);

	printk("Flagging Zephyr alive to host\n");
	swregs[3] = 0x12345600;
}

void arch_start_cpu(int cpu_num, k_thread_stack_t *stack, int sz,
		    arch_cpustart_t fn, void *arg)
{
	volatile struct soc_dsp_shim_regs *shim = (void *)SOC_DSP_SHIM_REG_BASE;
	volatile uint32_t *swregs = z_soc_uncached_ptr((void*)HP_SRAM_WIN0_BASE);
	uint32_t vecbase;

	if (!soc_mp_initialized) {
		soc_mp_initialize();
	}

	__ASSERT(cpu_num == 1, "Only supports only two CPUs!");

	/* Setup data to boot core #1 */
	__asm__ volatile("rsr.VECBASE %0\n\t" : "=r"(vecbase));

	start_rec.cpu = cpu_num;
	start_rec.fn = fn;
	start_rec.arg = arg;
	start_rec.vecbase = vecbase;
	start_rec.alive = 0;

	z_mp_stack_top = Z_THREAD_STACK_BUFFER(stack) + sz;

	printk("Waiting on host to flag CPU ready...\n");
	while ((swregs[3] & BIT(cpu_num)) == 0);
	k_busy_wait(50000);

	printk("Host says core %d is ready (ZSTAT 0x%x)...\n", cpu_num, swregs[3]);

	k_busy_wait(1000);
	printk("Disabling idle power gating on all cores\n");
	shim->pwrctl = 0xf | BIT(4) | BIT(6);

	printk("Setting up IDC interrupt\n");
	idcregs[0].core[cpu_num].ietc = ((long) z_soc_mp_asm_entry) >> 2;
	idcregs[0].core[cpu_num].itc = 0x01000002;
	printk("idcregs[%d].core[0].tfc = 0x%x\n", cpu_num, idcregs[cpu_num].core[0].tfc);
	printk("idcregs[%d].core[0].tefc = 0x%x\n", cpu_num, idcregs[cpu_num].core[0].tefc);

	printk("Triggering IDC interrupt @ %p\n", &idcregs[0].core[cpu_num].itc);
	idcregs[0].core[cpu_num].itc |= BIT(31); /* trigger interrupt */

	while(idcregs[cpu_num].core[0].tfc & BIT(31));

#if 0
	/* Enable IDC interrupt on the other core */
	printk("Enable interrupt in IDCCTL\n");
	idc_reg = idc_read(IPC_IDCCTL, cpu_num);
	idc_reg |= IPC_IDCCTL_IDCTBIE(0);
	idc_write(IPC_IDCCTL, cpu_num, idc_reg);
	/* FIXME: 8 is IRQ_BIT_LVL2_IDC / PLATFORM_IDC_INTERRUPT */
	printk("Enable L2 interrupt mask on other CPU\n");
	sys_set_bit(DT_REG_ADDR(DT_NODELABEL(cavs0)) + 0x04 +
		    CAVS_ICTL_INT_CPU_OFFSET(cpu_num), 8);

	/* Send power up message to the other core */
	printk("Send IDC to CPU\n");
	uint32_t ietc = IDC_MSG_POWER_UP_EXT((long) z_soc_mp_asm_entry);

	idc_write(IPC_IDCIETC(cpu_num), 0, ietc);
	idc_write(IPC_IDCITC(cpu_num), 0, IDC_MSG_POWER_UP | IPC_IDCITC_BUSY);

	/* Disable IDC interrupt on other core so IPI won't cause
	 * them to jump to ISR until the core is fully initialized.
	 */
	printk("Disable other side interrupt for now\n");
	idc_reg = idc_read(IPC_IDCCTL, cpu_num);
	idc_reg &= ~IPC_IDCCTL_IDCTBIE(0);
	idc_write(IPC_IDCCTL, cpu_num, idc_reg);
	sys_set_bit(DT_REG_ADDR(DT_NODELABEL(cavs0)) + 0x00 +
		      CAVS_ICTL_INT_CPU_OFFSET(cpu_num), 8);

	k_busy_wait(100);

#ifdef CONFIG_SMP_BOOT_DELAY
	printk("cavs_idc_smp_init()\n");
	cavs_idc_smp_init(NULL);
#endif

	/* Clear done bit from responding the power up message */
	printk("Clear DONE bit reading\n");
	idc_reg = idc_read(IPC_IDCIETC(cpu_num), 0) | IPC_IDCIETC_DONE;
	printk("Clear DONE bit writing\n");
	idc_write(IPC_IDCIETC(cpu_num), 0, idc_reg);
#endif

	printk("IDC sent, spinning...\n");
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
