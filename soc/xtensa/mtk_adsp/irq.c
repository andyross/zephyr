#include <zephyr/irq.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>

bool intc_mtk_adsp_get_enable(struct device *dev, int irq);
void intc_mtk_adsp_set_enable(struct device *dev, int irq, bool val);
bool intc_mtk_adsp_is_enabled(struct device *dev, int irq);

extern struct device DT_INST(0, mtk_adsp_intc);
extern struct device DT_INST(1, mtk_adsp_intc);

/* Sort of annoying: assumes there are exactly two controller devices
 * and that their instance IDs (i.e. the order in which they appear in
 * the .dts file) match their order in the _sw_isr_table[].  A better
 * scheme would be able to enumerate the tree at runtime.
 */
static struct device *irq_dev(unsigned int *irq_inout)
{
	if (*irq_inout < 64) {
		*irq_inout -= 32;
		return &DT_INST(0, mtk_adsp_intc);
	} else {
		*irq_inout -= 64;
		return &DT_INST(1, mtk_adsp_intc);
	}
}

void z_soc_irq_enable(unsigned int irq)
{
	if (irq < 32) {
		z_xtensa_irq_enable(irq);
	} else {
		struct device *dev = irq_dev(&irq);

		intc_mtk_adsp_set_enable(dev, irq, true);
	}
}

void z_soc_irq_disable(unsigned int irq)
{
	if (irq < 32) {
		z_xtensa_irq_disable(irq);
	} else {
		struct device *dev = irq_dev(&irq);

		intc_mtk_adsp_set_enable(dev, irq, false);
	}
}

int z_soc_irq_is_enabled(unsigned int irq)
{
	if (irq < 32) {
		return z_xtensa_irq_is_enabled(irq);
	} else {
		return intc_mtk_adsp_is_enabled(irq_dev(&irq), irq);
	}
}
