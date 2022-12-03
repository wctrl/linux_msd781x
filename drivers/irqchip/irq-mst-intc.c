// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Copyright (c) 2020 MediaTek Inc.
 * Author Mark-PK Tsai <mark-pk.tsai@mediatek.com>
 */
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/syscore_ops.h>

#define MST_INTC_MAX_IRQS	64

#define INTC_MASK		0x0
#define INTC_REV_POLARITY	0x10
#define INTC_EOI		0x20
#define INTC_PENDING		0x20

#ifdef CONFIG_PM_SLEEP
static LIST_HEAD(mst_intc_list);
#endif

struct mst_intc_chip_data {
	raw_spinlock_t	lock;
	unsigned int	irq_start, nr_irqs;
	void __iomem	*base;
	bool		no_eoi;
	bool		standalone;
	struct irq_domain *domain;
#ifdef CONFIG_PM_SLEEP
	struct list_head entry;
	u16 saved_polarity_conf[DIV_ROUND_UP(MST_INTC_MAX_IRQS, 16)];
#endif
};

static void mst_set_irq(struct irq_data *d, u32 offset)
{
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	struct mst_intc_chip_data *cd = irq_data_get_irq_chip_data(d);
	u16 val, mask;
	unsigned long flags;

	mask = 1 << (hwirq % 16);
	offset += (hwirq / 16) * 4;

	raw_spin_lock_irqsave(&cd->lock, flags);
	val = readw_relaxed(cd->base + offset) | mask;
	writew_relaxed(val, cd->base + offset);
	raw_spin_unlock_irqrestore(&cd->lock, flags);
}

static void mst_clear_irq(struct irq_data *d, u32 offset)
{
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	struct mst_intc_chip_data *cd = irq_data_get_irq_chip_data(d);
	u16 val, mask;
	unsigned long flags;

	mask = 1 << (hwirq % 16);
	offset += (hwirq / 16) * 4;

	raw_spin_lock_irqsave(&cd->lock, flags);
	val = readw_relaxed(cd->base + offset) & ~mask;
	writew_relaxed(val, cd->base + offset);
	raw_spin_unlock_irqrestore(&cd->lock, flags);
}

static void mst_intc_mask_irq(struct irq_data *d)
{
	struct mst_intc_chip_data *cd = irq_data_get_irq_chip_data(d);

	mst_set_irq(d, INTC_MASK);

	if (!cd->standalone)
		irq_chip_mask_parent(d);
}

static void mst_intc_unmask_irq(struct irq_data *d)
{
	struct mst_intc_chip_data *cd = irq_data_get_irq_chip_data(d);

	mst_clear_irq(d, INTC_MASK);

	if (!cd->standalone)
		irq_chip_unmask_parent(d);
}

static void mst_intc_eoi_irq(struct irq_data *d)
{
	struct mst_intc_chip_data *cd = irq_data_get_irq_chip_data(d);

	if (!cd->no_eoi)
		mst_set_irq(d, INTC_EOI);

	if (!cd->standalone)
		irq_chip_eoi_parent(d);
}

static int mst_irq_chip_set_type(struct irq_data *data, unsigned int type)
{
	struct mst_intc_chip_data *cd = irq_data_get_irq_chip_data(data);

	switch (type) {
	case IRQ_TYPE_LEVEL_LOW:
	case IRQ_TYPE_EDGE_FALLING:
		mst_set_irq(data, INTC_REV_POLARITY);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_EDGE_RISING:
		mst_clear_irq(data, INTC_REV_POLARITY);
		break;
	default:
		return -EINVAL;
	}

	if (cd->standalone)
		return 0;

	return irq_chip_set_type_parent(data, IRQ_TYPE_LEVEL_HIGH);
}

static void mst_intc_print_chip(struct irq_data *d, struct seq_file *p)
{
	struct mst_intc_chip_data *cd = irq_data_get_irq_chip_data(d);

	if (cd->no_eoi)
		seq_printf(p, "mst-irq");
	else
		seq_printf(p, "mst-fiq");

	if (cd->standalone)
		seq_printf(p, "-sa");
}

static struct irq_chip mst_intc_chip = {
	.name			= "mst-intc",
	.irq_print_chip		= mst_intc_print_chip,
	.irq_mask		= mst_intc_mask_irq,
	.irq_unmask		= mst_intc_unmask_irq,
	.irq_eoi		= mst_intc_eoi_irq,
	.irq_get_irqchip_state	= irq_chip_get_parent_state,
	.irq_set_irqchip_state	= irq_chip_set_parent_state,
	.irq_set_affinity	= irq_chip_set_affinity_parent,
	.irq_set_vcpu_affinity	= irq_chip_set_vcpu_affinity_parent,
	.irq_set_type		= mst_irq_chip_set_type,
	.irq_retrigger		= irq_chip_retrigger_hierarchy,
	.flags			= IRQCHIP_SET_TYPE_MASKED |
				  IRQCHIP_SKIP_SET_WAKE |
				  IRQCHIP_MASK_ON_SUSPEND,
};

static struct irq_chip mst_intc_sa_chip = {
	.name			= "mst-intc-sa",
	.irq_print_chip		= mst_intc_print_chip,
	.irq_mask		= mst_intc_mask_irq,
	.irq_unmask		= mst_intc_unmask_irq,
	.irq_eoi		= mst_intc_eoi_irq,
	.irq_set_type		= mst_irq_chip_set_type,
	.flags			= IRQCHIP_SET_TYPE_MASKED |
				  IRQCHIP_SKIP_SET_WAKE |
				  IRQCHIP_MASK_ON_SUSPEND,
};

static void mst_intc_chained_handler(struct irq_desc *desc)
{
	struct mst_intc_chip_data *cd = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	u16 hwirq, irq, i;

	chained_irq_enter(chip, desc);

	for (i = 0; i < DIV_ROUND_UP(cd->nr_irqs, 16); i++) {
		hwirq = readw_relaxed(cd->base + INTC_PENDING + i * 4);
		while (hwirq) {
			irq = __ffs(hwirq);
			hwirq &= ~BIT(irq);
			generic_handle_domain_irq(cd->domain, irq + i * 16);
		}
	}

	chained_irq_exit(chip, desc);
}

#ifdef CONFIG_PM_SLEEP
static void mst_intc_polarity_save(struct mst_intc_chip_data *cd)
{
	int i;
	void __iomem *addr = cd->base + INTC_REV_POLARITY;

	for (i = 0; i < DIV_ROUND_UP(cd->nr_irqs, 16); i++)
		cd->saved_polarity_conf[i] = readw_relaxed(addr + i * 4);
}

static void mst_intc_polarity_restore(struct mst_intc_chip_data *cd)
{
	int i;
	void __iomem *addr = cd->base + INTC_REV_POLARITY;

	for (i = 0; i < DIV_ROUND_UP(cd->nr_irqs, 16); i++)
		writew_relaxed(cd->saved_polarity_conf[i], addr + i * 4);
}

static void mst_irq_resume(void)
{
	struct mst_intc_chip_data *cd;

	list_for_each_entry(cd, &mst_intc_list, entry)
		mst_intc_polarity_restore(cd);
}

static int mst_irq_suspend(void)
{
	struct mst_intc_chip_data *cd;

	list_for_each_entry(cd, &mst_intc_list, entry)
		mst_intc_polarity_save(cd);
	return 0;
}

static struct syscore_ops mst_irq_syscore_ops = {
	.suspend	= mst_irq_suspend,
	.resume		= mst_irq_resume,
};

static int __init mst_irq_pm_init(void)
{
	register_syscore_ops(&mst_irq_syscore_ops);
	return 0;
}
late_initcall(mst_irq_pm_init);
#endif

static int mst_intc_domain_translate(struct irq_domain *d,
				     struct irq_fwspec *fwspec,
				     unsigned long *hwirq,
				     unsigned int *type)
{
	struct mst_intc_chip_data *cd = d->host_data;

	if (is_of_node(fwspec->fwnode)) {
		if (fwspec->param_count != 3)
			return -EINVAL;

		/* No PPI should point to this domain */
		if (fwspec->param[0] != 0)
			return -EINVAL;

		if (fwspec->param[1] >= cd->nr_irqs)
			return -EINVAL;

		*hwirq = fwspec->param[1];
		*type = fwspec->param[2] & IRQ_TYPE_SENSE_MASK;
		return 0;
	}

	return -EINVAL;
}

static int mst_intc_domain_alloc(struct irq_domain *domain, unsigned int virq,
				 unsigned int nr_irqs, void *data)
{
	int i;
	irq_hw_number_t hwirq;
	struct irq_fwspec parent_fwspec, *fwspec = data;
	struct mst_intc_chip_data *cd = domain->host_data;

	/* Not GIC compliant */
	if (fwspec->param_count != 3)
		return -EINVAL;

	/* No PPI should point to this domain */
	if (fwspec->param[0])
		return -EINVAL;

	hwirq = fwspec->param[1];
	for (i = 0; i < nr_irqs; i++)
		irq_domain_set_hwirq_and_chip(domain, virq + i, hwirq + i,
					      &mst_intc_chip,
					      domain->host_data);

	parent_fwspec = *fwspec;
	parent_fwspec.fwnode = domain->parent->fwnode;
	parent_fwspec.param[1] = cd->irq_start + hwirq;

	/*
	 * mst-intc latch the interrupt request if it's edge triggered,
	 * so the output signal to parent GIC is always level sensitive.
	 * And if the irq signal is active low, configure it to active high
	 * to meet GIC SPI spec in mst_irq_chip_set_type via REV_POLARITY bit.
	 */
	parent_fwspec.param[2] = IRQ_TYPE_LEVEL_HIGH;

	return irq_domain_alloc_irqs_parent(domain, virq, nr_irqs, &parent_fwspec);
}

static const struct irq_domain_ops mst_intc_domain_ops = {
	.translate	= mst_intc_domain_translate,
	.alloc		= mst_intc_domain_alloc,
	.free		= irq_domain_free_irqs_common,
};

static int mst_intc_domain_map(struct irq_domain *id, unsigned int virq,
				  irq_hw_number_t hw)
{
	struct mst_intc_chip_data *cd = id->host_data;

	irq_set_chip_data(virq, cd);

	/* TODO: maybe use edge-related stuff for FIQs */
	irq_set_chip_and_handler(virq, &mst_intc_sa_chip, handle_level_irq);
	irq_set_status_flags(virq, IRQ_LEVEL);

	irq_set_noprobe(virq);

	return 0;
}

static void mst_intc_domain_unmap(struct irq_domain *id, unsigned int virq)
{
	irq_set_chip_and_handler(virq, NULL, NULL);
}

static const struct irq_domain_ops mst_intc_sa_domain_ops = {
	.map    = mst_intc_domain_map,
	.unmap	= mst_intc_domain_unmap,
	.xlate  = irq_domain_xlate_twocell,
};

static int __init mst_intc_of_init(struct device_node *dn,
				   struct device_node *parent)
{
	struct irq_domain *domain, *domain_parent;
	struct mst_intc_chip_data *cd;
	u32 parent_irq, irq_start, irq_end, i;
	bool standalone = false;

	/* Try to get an interrupt line, as if we were a standalone intc */
	parent_irq = irq_of_parse_and_map(dn, 0);
	if (!parent_irq) {
		/* If failed, then try to get an interrupt parent instead */
		domain_parent = irq_find_host(parent);
		if (!domain_parent) {
			pr_err("mst-intc: interrupt-parent not found\n");
			return -EINVAL;
		}
	} else {
		/* If succeed, then from now on, we are the standalone intc */
		standalone = true;
	}

	if (of_property_read_u32_index(dn, "mstar,irqs-map-range", 0, &irq_start) ||
	    of_property_read_u32_index(dn, "mstar,irqs-map-range", 1, &irq_end))
		return -EINVAL;

	cd = kzalloc(sizeof(*cd), GFP_KERNEL);
	if (!cd)
		return -ENOMEM;

	cd->base = of_iomap(dn, 0);
	if (!cd->base) {
		kfree(cd);
		return -ENOMEM;
	}

	cd->standalone = standalone;
	cd->no_eoi = of_property_read_bool(dn, "mstar,intc-no-eoi");
	raw_spin_lock_init(&cd->lock);
	cd->irq_start = irq_start;
	cd->nr_irqs = irq_end - irq_start + 1;

	if (standalone) {
		/* A "standalone" intc handles all on its own. */
		domain = irq_domain_add_linear(dn, cd->nr_irqs,
						   &mst_intc_sa_domain_ops, cd);
	} else {
		/* A "non-standalone" intc wraps another intc. */
		domain = irq_domain_add_hierarchy(domain_parent, 0, cd->nr_irqs,
						  dn, &mst_intc_domain_ops, cd);
	}

	if (!domain) {
		iounmap(cd->base);
		kfree(cd);
		return -ENOMEM;
	}

	cd->domain = domain;

	/* Mask out all interrupts to avoid spurious ints */
	for (i = 0; i < DIV_ROUND_UP(cd->nr_irqs, 16); i++)
		writew_relaxed(0xffff, cd->base + INTC_MASK + i * 4);

	if (standalone) {
		/* A "standalone" intc exports a single irq line
		 * that has all incoming irqs ORed together.
		 */
		irq_set_chained_handler_and_data(parent_irq,
						  mst_intc_chained_handler, cd);
	}

#ifdef CONFIG_PM_SLEEP
	INIT_LIST_HEAD(&cd->entry);
	list_add_tail(&cd->entry, &mst_intc_list);
#endif
	return 0;
}

IRQCHIP_DECLARE(mst_intc, "mstar,mst-intc", mst_intc_of_init);
