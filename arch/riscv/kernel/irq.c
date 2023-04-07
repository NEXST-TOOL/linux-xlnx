// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2012 Regents of the University of California
 * Copyright (C) 2017 SiFive
 * Copyright (C) 2018 Christoph Hellwig
 */

#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/seq_file.h>
#include <asm/smp.h>

/*
 * Possible interrupt causes:
 */
#define INTERRUPT_CAUSE_SOFTWARE	IRQ_S_SOFT
#define INTERRUPT_CAUSE_TIMER		IRQ_S_TIMER
#define INTERRUPT_CAUSE_EXTERNAL	IRQ_S_EXT

extern int zynq_early_slcr_init(void);

int arch_show_interrupts(struct seq_file *p, int prec)
{
	show_ipi_stats(p, prec);
	return 0;
}

void __init init_IRQ(void)
{
	zynq_early_slcr_init();
	irqchip_init();
	if (!handle_arch_irq)
		panic("No interrupt controller found.");
}
