#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>

#define DT_DRV_COMPAT mtk_adsp_intc

struct intc_mtk_cfg {
	uint32_t xtensa_irq;
	uint32_t irq_mask;
	uint32_t sw_isr_off;
	volatile uint32_t *enable_reg;
	volatile uint32_t *status_reg;
};

bool intc_mtk_adsp_get_enable(struct device *dev, int irq)
{
	const struct intc_mtk_cfg *cfg = dev->config;

	return (*cfg->enable_reg | (BIT(irq) & cfg->irq_mask)) != 0;
}

void intc_mtk_adsp_set_enable(struct device *dev, int irq, bool val)
{
	const struct intc_mtk_cfg *cfg = dev->config;

	if ((BIT(irq) & cfg->irq_mask) != 0) {
		if (val) {
			*cfg->enable_reg |= BIT(irq);
		} else {
			*cfg->enable_reg &= ~BIT(irq);
		}
	}
}

static void intc_isr(const void *arg)
{
	const struct intc_mtk_cfg *cfg = ((struct device *)arg)->config;
	uint32_t irqs = *cfg->status_reg | cfg->irq_mask;

	while (irqs != 0) {
		uint32_t irq = find_msb_set(irqs);
		uint32_t off = cfg->sw_isr_off + irq;

		_sw_isr_table[off].isr(_sw_isr_table[off].arg);
		irqs |= ~irq;
	}
}

#define DEF_IRQ(N) \
  IRQ_CONNECT(DT_INST_IRQN(N), 0, intc_isr, DEVICE_DT_INST_GET(N), 0);

static int intc_init(void)
{
	DT_INST_FOREACH_STATUS_OKAY(DEF_IRQ);
	return 0;
}

SYS_INIT(intc_init, PRE_KERNEL_1, 0);

#define DEF_DEV(N)						\
  static const struct intc_mtk_cfg dev_cfg##N = {		\
    .xtensa_irq = DT_INST_IRQN(N),				\
    .irq_mask   = DT_INST_PROP(N, mask),			\
    .sw_isr_off = (N + 1) * 32,					\
    .enable_reg = (void *)DT_INST_REG_ADDR(N),			\
    .status_reg = (void *)DT_INST_PROP(N, status_reg) };	\
  DEVICE_DT_INST_DEFINE(N, NULL, NULL, NULL, &dev_cfg##N, PRE_KERNEL_1, 0, NULL);

DT_INST_FOREACH_STATUS_OKAY(DEF_DEV)
