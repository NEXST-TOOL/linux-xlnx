/*
 * PCIe host controller driver for Xilinx AXI PCIe Bridge
 *
 * Copyright (c) 2012 - 2014 Xilinx, Inc.
 *
 * Based on the Tegra PCIe driver
 *
 * Bits taken from Synopsys Designware Host controller driver and
 * ARM PCI Host generic driver.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of_address.h>
#include <linux/of_pci.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/pci.h>
#include <linux/platform_device.h>

/* Register definitions */
#define XILINX_PCIE_REG_BIR		0x00000130
#define XILINX_PCIE_REG_IDR		0x00000138
#define XILINX_PCIE_REG_IMR		0x0000013c
#define XILINX_PCIE_REG_PSCR		0x00000144
#define XILINX_PCIE_REG_RPSC		0x00000148
#define XILINX_PCIE_REG_MSIBASE1	0x0000014c
#define XILINX_PCIE_REG_MSIBASE2	0x00000150
#define XILINX_PCIE_REG_RPEFR		0x00000154
#define XILINX_PCIE_REG_RPIFR1		0x00000158
#define XILINX_PCIE_REG_RPIFR2		0x0000015c

/* Interrupt registers definitions */
#define XILINX_PCIE_INTR_LINK_DOWN	BIT(0)
#define XILINX_PCIE_INTR_ECRC_ERR	BIT(1)
#define XILINX_PCIE_INTR_STR_ERR	BIT(2)
#define XILINX_PCIE_INTR_HOT_RESET	BIT(3)
#define XILINX_PCIE_INTR_CFG_TIMEOUT	BIT(8)
#define XILINX_PCIE_INTR_CORRECTABLE	BIT(9)
#define XILINX_PCIE_INTR_NONFATAL	BIT(10)
#define XILINX_PCIE_INTR_FATAL		BIT(11)
#define XILINX_PCIE_INTR_INTX		BIT(16)
#define XILINX_PCIE_INTR_MSI		BIT(17)
#define XILINX_PCIE_INTR_SLV_UNSUPP	BIT(20)
#define XILINX_PCIE_INTR_SLV_UNEXP	BIT(21)
#define XILINX_PCIE_INTR_SLV_COMPL	BIT(22)
#define XILINX_PCIE_INTR_SLV_ERRP	BIT(23)
#define XILINX_PCIE_INTR_SLV_CMPABT	BIT(24)
#define XILINX_PCIE_INTR_SLV_ILLBUR	BIT(25)
#define XILINX_PCIE_INTR_MST_DECERR	BIT(26)
#define XILINX_PCIE_INTR_MST_SLVERR	BIT(27)
#define XILINX_PCIE_INTR_MST_ERRP	BIT(28)
#define XILINX_PCIE_IMR_ALL_MASK	0x1FF30FED
#define XILINX_PCIE_IDR_ALL_MASK	0xFFFFFFFF

/* Root Port Error FIFO Read Register definitions */
#define XILINX_PCIE_RPEFR_ERR_VALID	BIT(18)
#define XILINX_PCIE_RPEFR_REQ_ID	GENMASK(15, 0)
#define XILINX_PCIE_RPEFR_ALL_MASK	0xFFFFFFFF

/* Root Port Interrupt FIFO Read Register 1 definitions */
#define XILINX_PCIE_RPIFR1_INTR_VALID	BIT(31)
#define XILINX_PCIE_RPIFR1_MSI_INTR	BIT(30)
#define XILINX_PCIE_RPIFR1_INTR_MASK	GENMASK(28, 27)
#define XILINX_PCIE_RPIFR1_ALL_MASK	0xFFFFFFFF
#define XILINX_PCIE_RPIFR1_INTR_SHIFT	27

/* Bridge Info Register definitions */
#define XILINX_PCIE_BIR_ECAM_SZ_MASK	GENMASK(18, 16)
#define XILINX_PCIE_BIR_ECAM_SZ_SHIFT	16

/* Root Port Interrupt FIFO Read Register 2 definitions */
#define XILINX_PCIE_RPIFR2_MSG_DATA	GENMASK(15, 0)

/* Root Port Status/control Register definitions */
#define XILINX_PCIE_REG_RPSC_BEN	BIT(0)

/* Phy Status/Control Register definitions */
#define XILINX_PCIE_REG_PSCR_LNKUP	BIT(11)

/* ECAM definitions */
#define ECAM_BUS_NUM_SHIFT		20
#define ECAM_DEV_NUM_SHIFT		12

/* Number of MSI IRQs */
#define XILINX_NUM_MSI_IRQS		128

/**
 * struct xilinx_pcie_port - PCIe port information
 * @reg_base: IO Mapped Register Base
 * @list: port list
 * @pcie: pointer to PCIe host info
 * @msi_pages: MSI pages
 * @slot: Root port number
 * @leg_domain: Legacy INTx IRQ domain pointer
 * @msi_domain: MSI IRQ domain pointer
 * @msi_irq_in_use: bit map for assigned MSI IRQ 
 */
struct xilinx_pcie_port {
	void __iomem *reg_base;
	struct list_head list;
	struct xilinx_pcie *pcie;
	unsigned long msi_pages;
	u32 slot;
	struct irq_domain *msi_domain;
	struct irq_domain *leg_domain;
	DECLARE_BITMAP(msi_irq_in_use, XILINX_NUM_MSI_IRQS);
};

/**
 * struct xilinx_pcie - PCIe host (Root Complex) information
 * @dev: pointer to PCIe device
 * @io: IO resource
 * @pio: PIO resource
 * @mem: non-prefetchable memory resource
 * @busn: bus range
 * @offset: IO / Memory offset
 * @ports: pointer to PCIe port information
 * @res: pointer to I/O, memory and bus resources
 */
struct xilinx_pcie {
	struct device *dev;
	struct resource io;
	struct resource pio;
	struct resource mem;
	struct resource busn;
	struct {
		resource_size_t mem;
		resource_size_t io;
	} offset;
	struct list_head ports;
	struct list_head res;
};


static inline u32 pcie_read(struct xilinx_pcie_port *port, u32 reg)
{
	return readl(port->reg_base + reg);
}

static inline void pcie_write(struct xilinx_pcie_port *port, u32 val, u32 reg)
{
	writel(val, port->reg_base + reg);
}

static inline bool xilinx_pcie_link_is_up(struct xilinx_pcie_port *port)
{
	return (pcie_read(port, XILINX_PCIE_REG_PSCR) &
		XILINX_PCIE_REG_PSCR_LNKUP) ? 1 : 0;
}

/**
 * xilinx_pcie_clear_err_interrupts - Clear Error Interrupts
 * @port: PCIe port information
 */
static void xilinx_pcie_clear_err_interrupts(struct xilinx_pcie_port *port)
{
	unsigned long val = pcie_read(port, XILINX_PCIE_REG_RPEFR);

	if (val & XILINX_PCIE_RPEFR_ERR_VALID) {
		dev_dbg(port->pcie->dev, "Requester ID %lu\n",
			val & XILINX_PCIE_RPEFR_REQ_ID);
		pcie_write(port, XILINX_PCIE_RPEFR_ALL_MASK,
			   XILINX_PCIE_REG_RPEFR);
	}
}

static struct xilinx_pcie_port *xilinx_pcie_find_port(struct pci_bus *bus,
						unsigned int devfn)
{
	struct xilinx_pcie *pcie = bus->sysdata;
	struct xilinx_pcie_port *port;

	struct pci_bus *parent_bus;
	struct pci_dev *parent_bridge, *bridge = NULL;

	//traverse PCI tree first to determine which root port this device locates on
	parent_bridge = bus->self;
	while (parent_bridge != NULL)
	{
		parent_bus = parent_bridge->bus;
		bridge = parent_bridge;
		parent_bridge = parent_bus->self;
	}
	//bridge equals to NULL means current bus is root bus
	devfn = (bridge == NULL) ? devfn : (bridge->devfn);

	list_for_each_entry(port, &pcie->ports, list)
		if (port->slot == PCI_SLOT(devfn))
			return port;

	return NULL;
}

/**
 * xilinx_pcie_config_read - PCI RP/EP read operation
 * @bus: PCI Bus structure
 * @devfn: device/function
 * @where: address offset
 * @size: read size (might be 8-bit byte, 16-bit shord word, 32-bit long word and 64-bit quad word)
 * @val: read out result
 *
 * Return: 'true' on success and 'false' if invalid device is found
 */
static int xilinx_pcie_config_read(struct pci_bus *bus, unsigned int devfn,
				int where, int size, u32 *val)
{
	struct xilinx_pcie_port *port;

	int relbus;

	void __iomem* addr;

	port = xilinx_pcie_find_port(bus, devfn);
	if (!port) {
		*val = ~0;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	relbus = (bus->number << ECAM_BUS_NUM_SHIFT) | 
			((bus->number ? devfn : 0) << ECAM_DEV_NUM_SHIFT);

	addr = port->reg_base + relbus + where;

	if (size == 1)
		*val = readb(addr);
	else if (size == 2)
		*val = readw(addr);
	else
		*val = readl(addr);

	return PCIBIOS_SUCCESSFUL;
}

/* xilinx_pcie_config_read - PCI RP/EP read operation
 * @bus: PCI Bus structure
 * @devfn: device/function
 * @where: address offset
 * @size: write size (might be 8-bit byte, 16-bit shord word, 32-bit long word and 64-bit quad word)
 * @val: write data
 *
 * Return: 'true' on success and 'false' if invalid device is found
 */
static int xilinx_pcie_config_write(struct pci_bus *bus, unsigned int devfn,
				 int where, int size, u32 val)
{
	struct xilinx_pcie_port *port;
	int relbus;

	void __iomem* addr;

	port = xilinx_pcie_find_port(bus, devfn);
	if (!port)
		return PCIBIOS_DEVICE_NOT_FOUND;

	relbus = (bus->number << ECAM_BUS_NUM_SHIFT) | 
			((bus->number ? devfn : 0) << ECAM_DEV_NUM_SHIFT);

	addr = port->reg_base + relbus + where;

	if (size == 1)
		writeb(val, addr);
	else if (size == 2)
		writew(val, addr);
	else
		writel(val, addr);

	return PCIBIOS_SUCCESSFUL;
}

/* PCIe operations */
static struct pci_ops xilinx_pcie_ops = {
	//.map_bus = xilinx_pcie_map_bus,
	.read	= xilinx_pcie_config_read,
	.write	= xilinx_pcie_config_write,
};

/* MSI functions */

/**
 * xilinx_pcie_assign_msi - Allocate MSI number
 * @port: PCIe port structure
 *
 * Return: A valid IRQ on success and error value on failure.
 */
static int xilinx_pcie_assign_msi(struct xilinx_pcie_port *port)
{
	int pos;

	pos = find_first_zero_bit(port->msi_irq_in_use, XILINX_NUM_MSI_IRQS);
	if (pos < XILINX_NUM_MSI_IRQS)
		set_bit(pos, port->msi_irq_in_use);
	else
		return -ENOSPC;

	return pos;
}

/**
 * xilinx_msi_teardown_irq - Destroy the MSI
 * @chip: MSI Chip descriptor
 * @irq: MSI IRQ to destroy
 */
static void xilinx_msi_teardown_irq(struct msi_controller *chip,
				    unsigned int irq)
{
	struct pci_dev *pdev = to_pci_dev(chip->dev);
	struct irq_data *d = irq_get_irq_data(irq);
	irq_hw_number_t hwirq = irqd_to_hwirq(d);
	struct xilinx_pcie_port *port;

	port = xilinx_pcie_find_port(pdev->bus, pdev->devfn);
	if (!port)
		return;

	irq_dispose_mapping(irq);

	if (!test_bit(hwirq, port->msi_irq_in_use)) {
		dev_err(port->pcie->dev, "Trying to free unused MSI#%d\n", irq);
	} else {
		clear_bit(hwirq, port->msi_irq_in_use);
	}
}

/**
 * xilinx_pcie_msi_setup_irq - Setup MSI request
 * @chip: MSI chip pointer
 * @pdev: PCIe device pointer
 * @desc: MSI descriptor pointer
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_pcie_msi_setup_irq(struct msi_controller *chip,
				     struct pci_dev *pdev,
				     struct msi_desc *desc)
{
	struct xilinx_pcie_port *port;
	unsigned int irq;
	int hwirq;
	struct msi_msg msg;
	phys_addr_t msg_addr;

	port = xilinx_pcie_find_port(pdev->bus, pdev->devfn);
	if (!port)
		return -EINVAL;

	hwirq = xilinx_pcie_assign_msi(port);
	if (hwirq < 0)
		return hwirq;

	irq = irq_create_mapping(port->msi_domain, hwirq);
	if (!irq) {
		clear_bit(hwirq, port->msi_irq_in_use);
		return -EINVAL;
	}

	chip->dev = &pdev->dev;

	irq_set_msi_desc(irq, desc);

	msg_addr = virt_to_phys((void *)port->msi_pages);

	msg.address_hi = 0;
	msg.address_lo = lower_32_bits(msg_addr);
	msg.data = irq;

	pci_write_msi_msg(irq, &msg);

	return 0;
}

/* MSI Chip Descriptor */
static struct msi_controller xilinx_pcie_msi_chip = {
	.setup_irq = xilinx_pcie_msi_setup_irq,
	.teardown_irq = xilinx_msi_teardown_irq,
};

/* HW Interrupt Chip Descriptor */
static struct irq_chip xilinx_msi_irq_chip = {
	.name = "Xilinx PCIe MSI",
	.irq_enable = pci_msi_unmask_irq,
	.irq_disable = pci_msi_mask_irq,
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
};

/**
 * xilinx_pcie_msi_map - Set the handler for the MSI and mark IRQ as valid
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */
static int xilinx_pcie_msi_map(struct irq_domain *domain, unsigned int irq,
			       irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &xilinx_msi_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

/* IRQ Domain operations */
static const struct irq_domain_ops msi_domain_ops = {
	.map = xilinx_pcie_msi_map,
};

/**
 * xilinx_pcie_register_msi - Enable MSI support
 * @port: PCIe port information
 */
static void xilinx_pcie_register_msi(struct xilinx_pcie_port *port)
{
	phys_addr_t msg_addr;

	port->msi_pages = __get_free_pages(GFP_KERNEL, 0);
	msg_addr = virt_to_phys((void *)port->msi_pages);
	pcie_write(port, 0x0, XILINX_PCIE_REG_MSIBASE1);
	pcie_write(port, msg_addr, XILINX_PCIE_REG_MSIBASE2);
}

/* INTx Functions */

/**
 * xilinx_pcie_intx_map - Set the handler for the INTx and mark IRQ as valid
 * @domain: IRQ domain
 * @irq: Virtual IRQ number
 * @hwirq: HW interrupt number
 *
 * Return: Always returns 0.
 */
static int xilinx_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
				irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);

	return 0;
}

/* INTx IRQ Domain operations */
static const struct irq_domain_ops intx_domain_ops = {
	.map = xilinx_pcie_intx_map,
};

/* PCIe HW Functions */

/**
 * xilinx_pcie_intr_handler - Interrupt Service Handler
 * @irq: IRQ number
 * @data: PCIe port information
 *
 * Return: IRQ_HANDLED on success and IRQ_NONE on failure
 */
static irqreturn_t xilinx_pcie_intr_handler(int irq, void *data)
{
	struct xilinx_pcie_port *port = (struct xilinx_pcie_port *)data;
	u32 val, mask, status, msi_data;

	/* Read interrupt decode and mask registers */
	val = pcie_read(port, XILINX_PCIE_REG_IDR);
	mask = pcie_read(port, XILINX_PCIE_REG_IMR);

	status = val & mask;
	if (!status)
		return IRQ_NONE;

	if (status & XILINX_PCIE_INTR_LINK_DOWN)
		dev_warn(port->pcie->dev, "Link Down\n");

	if (status & XILINX_PCIE_INTR_ECRC_ERR)
		dev_warn(port->pcie->dev, "ECRC failed\n");

	if (status & XILINX_PCIE_INTR_STR_ERR)
		dev_warn(port->pcie->dev, "Streaming error\n");

	if (status & XILINX_PCIE_INTR_HOT_RESET)
		dev_info(port->pcie->dev, "Hot reset\n");

	if (status & XILINX_PCIE_INTR_CFG_TIMEOUT)
		dev_warn(port->pcie->dev, "ECAM access timeout\n");

	if (status & XILINX_PCIE_INTR_CORRECTABLE) {
		dev_warn(port->pcie->dev, "Correctable error message\n");
		xilinx_pcie_clear_err_interrupts(port);
	}

	if (status & XILINX_PCIE_INTR_NONFATAL) {
		dev_warn(port->pcie->dev, "Non fatal error message\n");
		xilinx_pcie_clear_err_interrupts(port);
	}

	if (status & XILINX_PCIE_INTR_FATAL) {
		dev_warn(port->pcie->dev, "Fatal error message\n");
		xilinx_pcie_clear_err_interrupts(port);
	}

	if (status & XILINX_PCIE_INTR_INTX) {
		/* INTx interrupt received */
		val = pcie_read(port, XILINX_PCIE_REG_RPIFR1);

		/* Check whether interrupt valid */
		if (!(val & XILINX_PCIE_RPIFR1_INTR_VALID)) {
			dev_warn(port->pcie->dev, "RP Intr FIFO1 read error\n");
			goto error;
		}

		if (!(val & XILINX_PCIE_RPIFR1_MSI_INTR)) {
			/* Clear interrupt FIFO register 1 */
			pcie_write(port, XILINX_PCIE_RPIFR1_ALL_MASK,
				   XILINX_PCIE_REG_RPIFR1);

			/* Handle INTx Interrupt */
			val = ((val & XILINX_PCIE_RPIFR1_INTR_MASK) >>
				XILINX_PCIE_RPIFR1_INTR_SHIFT) + 1;
			generic_handle_irq(irq_find_mapping(port->leg_domain,
							    val));
		}
	}

	if (status & XILINX_PCIE_INTR_MSI) {
		/* MSI Interrupt */
		val = pcie_read(port, XILINX_PCIE_REG_RPIFR1);

		if (!(val & XILINX_PCIE_RPIFR1_INTR_VALID)) {
			dev_warn(port->pcie->dev, "RP Intr FIFO1 read error\n");
			goto error;
		}

		if (val & XILINX_PCIE_RPIFR1_MSI_INTR) {
			msi_data = pcie_read(port, XILINX_PCIE_REG_RPIFR2) &
				   XILINX_PCIE_RPIFR2_MSG_DATA;

			/* Clear interrupt FIFO register 1 */
			pcie_write(port, XILINX_PCIE_RPIFR1_ALL_MASK,
				   XILINX_PCIE_REG_RPIFR1);

			if (IS_ENABLED(CONFIG_PCI_MSI)) {
				/* Handle MSI Interrupt */
				generic_handle_irq(msi_data);
			}
		}
	}

	if (status & XILINX_PCIE_INTR_SLV_UNSUPP)
		dev_warn(port->pcie->dev, "Slave unsupported request\n");

	if (status & XILINX_PCIE_INTR_SLV_UNEXP)
		dev_warn(port->pcie->dev, "Slave unexpected completion\n");

	if (status & XILINX_PCIE_INTR_SLV_COMPL)
		dev_warn(port->pcie->dev, "Slave completion timeout\n");

	if (status & XILINX_PCIE_INTR_SLV_ERRP)
		dev_warn(port->pcie->dev, "Slave Error Poison\n");

	if (status & XILINX_PCIE_INTR_SLV_CMPABT)
		dev_warn(port->pcie->dev, "Slave Completer Abort\n");

	if (status & XILINX_PCIE_INTR_SLV_ILLBUR)
		dev_warn(port->pcie->dev, "Slave Illegal Burst\n");

	if (status & XILINX_PCIE_INTR_MST_DECERR)
		dev_warn(port->pcie->dev, "Master decode error\n");

	if (status & XILINX_PCIE_INTR_MST_SLVERR)
		dev_warn(port->pcie->dev, "Master slave error\n");

	if (status & XILINX_PCIE_INTR_MST_ERRP)
		dev_warn(port->pcie->dev, "Master error poison\n");

error:
	/* Clear the Interrupt Decode register */
	pcie_write(port, status, XILINX_PCIE_REG_IDR);

	return IRQ_HANDLED;
}

/**
 * xilinx_pcie_free_irq_domain - Free IRQ domain
 * @port: PCIe port information
 */
static void xilinx_pcie_free_irq_domain(struct xilinx_pcie_port *port)
{
	int i;
	u32 irq, num_irqs;

	/* Free IRQ Domain */
	if (IS_ENABLED(CONFIG_PCI_MSI)) {

		free_pages(port->msi_pages, 0);

		num_irqs = XILINX_NUM_MSI_IRQS;
	} else {
		/* INTx */
		num_irqs = 4;
	}

	for (i = 0; i < num_irqs; i++) {
		irq = irq_find_mapping(port->leg_domain, i + 1);
		if (irq > 0)
			irq_dispose_mapping(irq);
	}
	if (port->leg_domain)
		irq_domain_remove(port->leg_domain);
	if (port->msi_domain)
		irq_domain_remove(port->msi_domain);
}

/**
 * xilinx_pcie_init_irq_domain - Initialize IRQ domain
 * @port: PCIe port information
 * @node: Device Tree node to parse
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_pcie_init_irq_domain(struct xilinx_pcie_port *port,
					struct device_node *node)
{
	struct device *dev = port->pcie->dev;
	struct device_node *pcie_intc_node;

	/* Setup INTx */
	pcie_intc_node = of_get_next_child(node, NULL);
	if (!pcie_intc_node) {
		dev_err(dev, "No PCIe Intc node found\n");
		return PTR_ERR(pcie_intc_node);
	}

	port->leg_domain = irq_domain_add_linear(pcie_intc_node, 4,
						 &intx_domain_ops,
						 port);
	if (!port->leg_domain) {
		dev_err(dev, "Failed to get a INTx IRQ domain\n");
		return PTR_ERR(port->leg_domain);
	}

	/* Setup MSI */
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		port->msi_domain = irq_domain_add_linear(node,
							 XILINX_NUM_MSI_IRQS,
							 &msi_domain_ops,
							 &xilinx_pcie_msi_chip);
		if (!port->msi_domain) {
			dev_err(dev, "Failed to get a MSI IRQ domain\n");
			return PTR_ERR(port->msi_domain);
		}

		xilinx_pcie_register_msi(port);
	}

	return 0;
}

/**
 * xilinx_pcie_register - register tree-like PCIe bridges, buses and devices
 * @pcie: Xilinx PCIe structure
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_pcie_register(struct xilinx_pcie *pcie)
{
	struct pci_bus *bus;
	bus = pci_create_root_bus(pcie->dev, 0,
				  &xilinx_pcie_ops, pcie, &pcie->res);
	if (!bus)
		return -ENOMEM;

#ifdef CONFIG_PCI_MSI
	xilinx_pcie_msi_chip.dev = pcie->dev;
	bus->msi = &xilinx_pcie_msi_chip;
#endif
	pci_scan_child_bus(bus);
	pci_assign_unassigned_bus_resources(bus);
#ifndef CONFIG_MICROBLAZE
	pci_fixup_irqs(pci_common_swizzle, of_irq_parse_and_map_pci);
#endif
	pci_bus_add_devices(bus);

	return 0;
}

static void xilinx_pcie_port_free(struct xilinx_pcie_port *port)
{
	struct xilinx_pcie *pcie = port->pcie;
	struct device *dev = pcie->dev;

	devm_iounmap(dev, port->reg_base);
	list_del(&port->list);
	devm_kfree(dev, port);
}

static void xilinx_pcie_put_resources(struct xilinx_pcie *pcie)
{
	struct xilinx_pcie_port *port, *tmp;

	list_for_each_entry_safe(port, tmp, &pcie->ports, list) {
		xilinx_pcie_free_irq_domain(port);
		xilinx_pcie_port_free(port);
	}
}

/**
 * xilinx_pcie_request_resources - add resources to list
 * @pcie: Xilinx PCIe structure
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_pcie_request_resources(struct xilinx_pcie *pcie)
{
	//struct device *dev = pcie->dev;
	//int err;

	INIT_LIST_HEAD(&pcie->res);

	pci_add_resource_offset(&pcie->res, &pcie->pio, pcie->offset.io);
	pci_add_resource_offset(&pcie->res, &pcie->mem, pcie->offset.mem);
	pci_add_resource(&pcie->res, &pcie->busn);

	/*err = devm_request_pci_bus_resources(dev, &pcie->res);
	if (err < 0)
		return err;*/

	pci_remap_iospace(&pcie->pio, pcie->io.start);

	return 0;
}

/**
 * xilinx_pcie_init_port - Initialize hardware
 * @port: PCIe port information
 */
static void xilinx_pcie_init_port(struct xilinx_pcie_port *port)
{
	if (!xilinx_pcie_link_is_up(port))
	{
		dev_info(port->pcie->dev, "PCIe Link is DOWN\n");
		xilinx_pcie_port_free(port);
		return;
	}

	dev_info(port->pcie->dev, "PCIe Link is UP\n");

	/* Disable all interrupts */
	pcie_write(port, ~XILINX_PCIE_IDR_ALL_MASK,
		   XILINX_PCIE_REG_IMR);

	/* Clear pending interrupts */
	pcie_write(port, pcie_read(port, XILINX_PCIE_REG_IDR) &
			 XILINX_PCIE_IMR_ALL_MASK,
		   XILINX_PCIE_REG_IDR);

	/* Enable all interrupts */
	pcie_write(port, XILINX_PCIE_IMR_ALL_MASK, XILINX_PCIE_REG_IMR);

	/* Enable the Bridge enable bit */
	pcie_write(port, pcie_read(port, XILINX_PCIE_REG_RPSC) |
			 XILINX_PCIE_REG_RPSC_BEN,
		   XILINX_PCIE_REG_RPSC);
}

/**
 * xilinx_pcie_setup_irq - Setup INTx/MSI interrupt for each root port
 * @port: PCIe root port information
 * @node: Device Tree Node to parse
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_pcie_setup_irq(struct xilinx_pcie_port *port,
			      struct device_node *node)
{
	struct xilinx_pcie *pcie = port->pcie;
	struct device *dev = pcie->dev;
	struct platform_device *pdev = to_platform_device(dev);
	int err, irq;

	irq = platform_get_irq(pdev, port->slot);
	err = devm_request_irq(dev, irq, xilinx_pcie_intr_handler,
			       IRQF_SHARED, "xilinx-pcie", port);
	if (err) {
		dev_err(dev, "unable to request IRQ %d\n", irq);
		return err;
	}

	err = xilinx_pcie_init_irq_domain(port, node);
	if (err) {
		dev_err(dev, "failed to init PCIe IRQ domain\n");
		return err;
	}

	return 0;
}

/**
 * xilinx_pcie_parse_port - Parse Root Port
 * @pcie: PCIe root complex information
 * @node: Device Tree Node to parse
 * @slot: the number of current root port
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_pcie_parse_port(struct xilinx_pcie *pcie,
			       struct device_node *node,
			       int slot)
{
	struct xilinx_pcie_port *port;
	struct resource *regs;
	struct device *dev = pcie->dev;
	struct platform_device *pdev = to_platform_device(dev);
	char name[10];
	int err;

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	snprintf(name, sizeof(name), "rp%d", slot);
	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	port->reg_base = devm_ioremap_resource(dev, regs);
	if (IS_ERR(port->reg_base)) {
		dev_err(dev, "failed to map port%d base\n", slot);
		return PTR_ERR(port->reg_base);
	}

	port->slot = slot;
	port->pcie = pcie;

	/*setup interrupt*/
	err = xilinx_pcie_setup_irq(port, node);
	if (err)
		return err;

	INIT_LIST_HEAD(&port->list);
	list_add_tail(&port->list, &pcie->ports);

	return 0;
}

/**
 * xilinx_pcie_parse_dt - Parse Device tree and setup root ports
 * @pcie: PCIe root complex information
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_pcie_parse_dt(struct xilinx_pcie *pcie)
{
	struct device *dev = pcie->dev;
	struct device_node *node = dev->of_node, *child;

	struct of_pci_range_parser parser;
	struct of_pci_range range;

	struct resource res;
	struct xilinx_pcie_port *port, *tmp;

	const char *type;
	int err;

	/*parse device_type (this field must be pci)*/
	type = of_get_property(node, "device_type", NULL);
	if (!type || strcmp(type, "pci")) {
		dev_err(dev, "invalid \"device_type\" %s\n", type);
		return -EINVAL;
	}

	/*parse range field for PCI I/O and Memory region*/
	if (of_pci_range_parser_init(&parser, node)) {
		dev_err(dev, "missing \"ranges\" property\n");
		return -EINVAL;
	}

	for_each_of_pci_range(&parser, &range) {
		err = of_pci_range_to_resource(&range, node, &res);
		if (err < 0)
			return err;

		switch (res.flags & IORESOURCE_TYPE_BITS) {
		case IORESOURCE_IO:
			pcie->offset.io = res.start - range.pci_addr;

			memcpy(&pcie->pio, &res, sizeof(res));
			pcie->pio.name = node->full_name;

			pcie->io.start = range.cpu_addr;
			pcie->io.end = range.cpu_addr + range.size - 1;
			pcie->io.flags = IORESOURCE_MEM;
			pcie->io.name = "I/O";

			memcpy(&res, &pcie->io, sizeof(res));
			break;

		case IORESOURCE_MEM:
			pcie->offset.mem = res.start - range.pci_addr;

			memcpy(&pcie->mem, &res, sizeof(res));
			pcie->mem.name = "non-prefetchable";
			break;
		}
	}

	/*parse optional bus_range field*/
	err = of_pci_parse_bus_range(node, &pcie->busn);
	if (err < 0) {
		dev_err(dev, "failed to parse bus ranges property: %d\n", err);
		pcie->busn.name = node->name;
		pcie->busn.start = 0;
		pcie->busn.end = 0xff;
		pcie->busn.flags = IORESOURCE_BUS;
	}

	/*parse root ports declared in device tree*/
	for_each_available_child_of_node(node, child) {
		int slot;

		/*parse the reg field of child node to obtain the slot number of root port*/
		err = of_pci_get_devfn(child);
		if (err < 0) {
			dev_err(dev, "failed to parse devfn: %d\n", err);
			return err;
		}

		slot = PCI_SLOT(err);

		err = xilinx_pcie_parse_port(pcie, child, slot);
		if (err)
			return err;
	}

	/* enable each port, and then check link status */
	list_for_each_entry_safe(port, tmp, &pcie->ports, list)
		xilinx_pcie_init_port(port);

	return 0;
}

/**
 * xilinx_pcie_probe - Probe function
 * @pdev: Platform device pointer
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct xilinx_pcie *pcie;

	int err;

	/*check if Device Tree Node is available*/
	if (!dev->of_node)
		return -ENODEV;

	/*allocate memory space for local struct -- xilinx_pcie*/
	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->dev = dev;
	platform_set_drvdata(pdev, pcie);
	INIT_LIST_HEAD(&pcie->ports);

	/*setup root complex and root ports by parsing device tree*/
	err = xilinx_pcie_parse_dt(pcie);
	if (err) {
		dev_err(dev, "Parsing DT failed\n");
		return err;
	}

	/*Add resources of I/O, memory and bus*/
	err = xilinx_pcie_request_resources(pcie);
	if (err)
		goto put_resources;

	/*register tree-like PCIe resources*/
	err = xilinx_pcie_register(pcie);
	if (err)
		goto put_resources;

	return 0;

put_resources:
	if (!list_empty(&pcie->ports))
		xilinx_pcie_put_resources(pcie);

	return err;
}

/**
 * xilinx_pcie_remove - Remove function
 * @pdev: Platform device pointer
 *
 * Return: '0' always
 */
static int xilinx_pcie_remove(struct platform_device *pdev)
{
	struct xilinx_pcie *pcie = platform_get_drvdata(pdev);

	if (!list_empty(&pcie->ports))
		xilinx_pcie_put_resources(pcie);

	return 0;
}

static struct of_device_id xilinx_pcie_of_match[] = {
	{ .compatible = "xlnx,axi-pcie-host-1.00.a", },
	{}
};

static struct platform_driver xilinx_pcie_driver = {
	.driver = {
		.name = "xilinx-pcie",
		.of_match_table = xilinx_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = xilinx_pcie_probe,
	.remove = xilinx_pcie_remove,
};
module_platform_driver(xilinx_pcie_driver);

MODULE_AUTHOR("Xilinx Inc");
MODULE_DESCRIPTION("Xilinx AXI PCIe driver");
MODULE_LICENSE("GPL v2");

