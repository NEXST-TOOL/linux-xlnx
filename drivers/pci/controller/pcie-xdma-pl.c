// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCIe host controller driver for Xilinx XDMA PCIe Bridge
 *
 * Copyright (C) 2017 Xilinx, Inc. All rights reserved.
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
#include <linux/irqchip/chained_irq.h>
#include "../pci.h"

/* Register definitions */
#define XILINX_PCIE_REG_VSEC		0x0000012c
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
#define XILINX_PCIE_REG_IDRN            0x00000160
#define XILINX_PCIE_REG_IDRN_MASK       0x00000164
#define XILINX_PCIE_REG_MSI_LOW		0x00000170
#define XILINX_PCIE_REG_MSI_HI		0x00000174
#define XILINX_PCIE_REG_MSI_LOW_MASK	0x00000178
#define XILINX_PCIE_REG_MSI_HI_MASK	0x0000017c

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
#define XILINX_PCIE_IDRN_MASK           GENMASK(19, 16)

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
#define XILINX_PCIE_IDRN_SHIFT          16
#define XILINX_PCIE_VSEC_REV_MASK	GENMASK(19, 16)
#define XILINX_PCIE_VSEC_REV_SHIFT	16
#define XILINX_PCIE_FIFO_SHIFT		5

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
#define XILINX_NUM_MSI_IRQS		64
#define INTX_NUM                        4
#define MSI_GIC_AFFINITY                 2
#define MAXIMUM_HWIRQ_NUM               32

#define DMA_BRIDGE_BASE_OFF		0xCD8

enum msi_mode {
	MSI_DECD_MODE = 1,
	MSI_FIFO_MODE,
};

enum xdma_config {
	XDMA_ZYNQMP_PL = 1,
	XDMA_VERSAL_PL,
};

enum msi_decd_mode_order {
	REQ_DECD_MISC = 0,
	REQ_DECD_MSI0,
	REQ_DECD_MSI1,
	REQ_DECD_TOTAL,
};

struct xilinx_msi {
	unsigned long *msi_irq_in_use;
	struct mutex lock;		/* protect bitmap variable */
	unsigned long msi_pages;
	phys_addr_t msi_phys;
};

/**
 * struct xilinx_pcie_port - PCIe port information
 * @reg_base: IO Mapped Register Base
 * @list: port list
 * @pcie: pointer to PCIe host info
 * @irq: Interrupt number
 * @slot: Root Port number
 * @dev: Device pointer
 * @leg_domain: Legacy IRQ domain pointer
 * @msi: MSI information
 * @msi_mode: MSI mode
 * @xdma_config: XDMA IP configuration
 */
struct xilinx_pcie_port {
	void __iomem *reg_base;
	struct list_head list;
	struct xilinx_pcie *pcie;
	u32 irq;
	u32 slot;
	struct device *dev;
	struct irq_domain *leg_domain;
	struct xilinx_msi msi;
	u8 msi_mode;
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
	int nr_cpus;
	struct irq_domain *msi_domain;
	u8 xdma_config;
};

static inline u32 pcie_read(struct xilinx_pcie_port *port, u32 reg)
{
	struct xilinx_pcie *pcie = port->pcie;

	if (pcie->xdma_config == XDMA_ZYNQMP_PL)
		return readl(port->reg_base + reg);
	else
		return readl(port->reg_base + reg + DMA_BRIDGE_BASE_OFF);
}

static inline void pcie_write(struct xilinx_pcie_port *port, u32 val, u32 reg)
{
	struct xilinx_pcie *pcie = port->pcie;

	if (pcie->xdma_config == XDMA_ZYNQMP_PL)
		writel(val, port->reg_base + reg);
	else
		writel(val, port->reg_base + reg + DMA_BRIDGE_BASE_OFF);
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
		dev_dbg(port->dev, "Requester ID %lu\n",
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

/* xilinx_pcie_config_write - PCI RP/EP write operation
 * @bus: PCI Bus structure
 * @devfn: device/function
 * @where: address offset
 * @size: write size (might be 8-bit byte, 16-bit short word, 32-bit long word and 64-bit quad word)
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
	.read	= xilinx_pcie_config_read,
	.write	= xilinx_pcie_config_write,
};

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
	irq_set_status_flags(irq, IRQ_LEVEL);

	return 0;
}

/* INTx IRQ Domain operations */
static const struct irq_domain_ops intx_domain_ops = {
	.map = xilinx_pcie_intx_map,
	.xlate = pci_irqd_intx_xlate,
};

/* PCIe INTx handler */

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
	u32 val, mask, status, bit;
	unsigned long intr_val;

	/* Read interrupt decode and mask registers */
	val = pcie_read(port, XILINX_PCIE_REG_IDR);
	mask = pcie_read(port, XILINX_PCIE_REG_IMR);

	status = val & mask;
	if (!status)
		return IRQ_NONE;

	if (status & XILINX_PCIE_INTR_LINK_DOWN)
		dev_warn(port->dev, "Link Down\n");

	if (status & XILINX_PCIE_INTR_ECRC_ERR)
		dev_warn(port->dev, "ECRC failed\n");

	if (status & XILINX_PCIE_INTR_STR_ERR)
		dev_warn(port->dev, "Streaming error\n");

	if (status & XILINX_PCIE_INTR_HOT_RESET)
		dev_info(port->dev, "Hot reset\n");

	if (status & XILINX_PCIE_INTR_CFG_TIMEOUT)
		dev_warn(port->dev, "ECAM access timeout\n");

	if (status & XILINX_PCIE_INTR_CORRECTABLE) {
		dev_warn(port->dev, "Correctable error message\n");
		xilinx_pcie_clear_err_interrupts(port);
	}

	if (status & XILINX_PCIE_INTR_NONFATAL) {
		dev_warn(port->dev, "Non fatal error message\n");
		xilinx_pcie_clear_err_interrupts(port);
	}

	if (status & XILINX_PCIE_INTR_FATAL) {
		dev_warn(port->dev, "Fatal error message\n");
		xilinx_pcie_clear_err_interrupts(port);
	}

	if (status & XILINX_PCIE_INTR_INTX) {
		/* Handle INTx Interrupt */
		intr_val = pcie_read(port, XILINX_PCIE_REG_IDRN);
		intr_val = intr_val >> XILINX_PCIE_IDRN_SHIFT;

		for_each_set_bit(bit, &intr_val, INTX_NUM)
			generic_handle_irq(irq_find_mapping(port->leg_domain,
							    bit));
	}

	if (status & XILINX_PCIE_INTR_SLV_UNSUPP)
		dev_warn(port->dev, "Slave unsupported request\n");

	if (status & XILINX_PCIE_INTR_SLV_UNEXP)
		dev_warn(port->dev, "Slave unexpected completion\n");

	if (status & XILINX_PCIE_INTR_SLV_COMPL)
		dev_warn(port->dev, "Slave completion timeout\n");

	if (status & XILINX_PCIE_INTR_SLV_ERRP)
		dev_warn(port->dev, "Slave Error Poison\n");

	if (status & XILINX_PCIE_INTR_SLV_CMPABT)
		dev_warn(port->dev, "Slave Completer Abort\n");

	if (status & XILINX_PCIE_INTR_SLV_ILLBUR)
		dev_warn(port->dev, "Slave Illegal Burst\n");

	if (status & XILINX_PCIE_INTR_MST_DECERR)
		dev_warn(port->dev, "Master decode error\n");

	if (status & XILINX_PCIE_INTR_MST_SLVERR)
		dev_warn(port->dev, "Master slave error\n");

	if (status & XILINX_PCIE_INTR_MST_ERRP)
		dev_warn(port->dev, "Master error poison\n");

	return IRQ_HANDLED;
}

/* MSI Interrupt */

static int hwirq_to_cpu(unsigned long hwirq)
{
	return (hwirq % MSI_GIC_AFFINITY);
}

static unsigned long hwirq_to_canonical_hwirq(unsigned long hwirq)
{
	return hwirq - hwirq_to_cpu(hwirq);
}

static void xilinx_pcie_handle_msi_irq(struct xilinx_pcie_port *port,
				       u32 status_reg)
{
	struct xilinx_msi *msi;
	unsigned long status;
	u32 bit;
	u32 virq;

	msi = &port->msi;

	while ((status = pcie_read(port, status_reg)) != 0) {
		for_each_set_bit(bit, &status, 32) {
			pcie_write(port, 1 << bit, status_reg);
			virq = irq_find_mapping(port->pcie->msi_domain, 
			           (bit + (XILINX_NUM_MSI_IRQS * port->slot)));
			if (virq)
			        generic_handle_irq(virq);
		}
	}
}

static void xilinx_pcie_msi_handler_high(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct xilinx_pcie_port *port = irq_desc_get_handler_data(desc);

	chained_irq_enter(chip, desc);
	xilinx_pcie_handle_msi_irq(port, XILINX_PCIE_REG_MSI_HI);
	chained_irq_exit(chip, desc);
}

static void xilinx_pcie_msi_handler_low(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct xilinx_pcie_port *port = irq_desc_get_handler_data(desc);

	chained_irq_enter(chip, desc);
	xilinx_pcie_handle_msi_irq(port, XILINX_PCIE_REG_MSI_LOW);
	chained_irq_exit(chip, desc);
}

static int xilinx_pcie_msi_prepare(struct irq_domain *domain, 
                                   struct device *dev, int nvec, msi_alloc_info_t *arg)
{
	memset(arg, 0, sizeof(*arg));
	arg->scratchpad[0].ptr = to_pci_dev(dev);
	
	return 0;
}

static irq_hw_number_t 
xilinx_pcie_msi_get_hwirq(struct msi_domain_info *info, msi_alloc_info_t *arg)
{
	struct pci_dev *pdev = (struct pci_dev *)arg->scratchpad[0].ptr; 
	struct xilinx_pcie_port *port;
	struct xilinx_msi *msi;
	irq_hw_number_t pos;
	
	/*find PCIe port for such device*/
	port = xilinx_pcie_find_port(pdev->bus, pdev->devfn);
	if(!port)
	        return -EINVAL;
		
	msi = &port->msi;
	
	/*find available hwirq number*/
	mutex_lock(&msi->lock);
	pos = bitmap_find_next_zero_area(msi->msi_irq_in_use, 
	        XILINX_NUM_MSI_IRQS, 0, MSI_GIC_AFFINITY, 0);

	// currently only 16 MSI IRQs (#0 - #15) are available in each port
	// and each MSI IRQ occupies adjacent hwirq numbers 
	// (e.g., MSI IRQ 0 occupies hwirq 0 and 1), mainly for CPU affinity
	// As a result, at most 32 hwirq numbers are allowed for each port
	if (pos >= MAXIMUM_HWIRQ_NUM) {
		mutex_unlock(&msi->lock);
		return -ENOSPC;
	}
	
	// make sure that the occupied hwirq numbers are aligned to MSI_GIC_AFFINITY
	bitmap_set(msi->msi_irq_in_use, pos, MSI_GIC_AFFINITY);
	
	// mainly used in case of MSI_GIC_AFFINITY == 2
	if (pos && (!((pos >> 1) & 0x01)))
	        pos++;
	    
	mutex_unlock(&msi->lock);
	
	/*maintaining MSI IRQs of each port in the root bus MSI domain*/
	return (pos + (XILINX_NUM_MSI_IRQS * port->slot));
}

static void xilinx_pcie_msi_free(struct irq_domain *domain, 
    struct msi_domain_info *info, unsigned int virq)
{
	struct irq_data *data = irq_domain_get_irq_data(domain, virq);
	struct msi_desc *desc = irq_data_get_msi_desc(data);
	struct pci_dev *pdev = to_pci_dev(desc->dev); 

	struct xilinx_pcie_port *port;
	struct xilinx_msi *msi;

	irq_hw_number_t pos;
	
	port = xilinx_pcie_find_port(pdev->bus, pdev->devfn);
	if(!port)
	        return;
		
	msi = &port->msi;
	pos = data->hwirq - (XILINX_NUM_MSI_IRQS * port->slot);
	// obeying the same rule of CPU affinity as in get_hwirq() func.
	pos -= (pos & 0x01);
	
	mutex_lock(&msi->lock);	
	bitmap_clear(msi->msi_irq_in_use, pos, MSI_GIC_AFFINITY);
	mutex_unlock(&msi->lock);
}

static void xilinx_compose_msi_msg(struct irq_data *data, struct msi_msg *msg)
{
	struct msi_desc *desc = irq_data_get_msi_desc(data);
	struct pci_dev *pdev = to_pci_dev(desc->dev); 

	struct xilinx_pcie_port *port;
	struct xilinx_msi *msi;
	
	port = xilinx_pcie_find_port(pdev->bus, pdev->devfn);
	if(!port)
	        return;
		
	msi = &port->msi;

	msg->address_lo = lower_32_bits(msi->msi_phys);
	msg->address_hi = upper_32_bits(msi->msi_phys);
	// guarantee that MSI IRQs with odd hwirq number 
	// would invoke interrupt signal 
	// via the vector32to63 port 
	// that is bounden to CPU core #1 for affinity 
	msg->data = data->hwirq + ((data->hwirq & 0x1) << 5);
}

static int xilinx_msi_set_affinity(struct irq_data *irq_data,
				   const struct cpumask *mask, bool force)
{
	int target_cpu = cpumask_first(mask) % MSI_GIC_AFFINITY;
	int curr_cpu;
	
	curr_cpu = hwirq_to_cpu(irq_data->hwirq);
	if(curr_cpu == target_cpu)
	        return IRQ_SET_MASK_OK_DONE;
		
	/*Update MSI number to target the new CPU*/
	irq_data->hwirq = hwirq_to_canonical_hwirq(irq_data->hwirq) + target_cpu;
	
	return IRQ_SET_MASK_OK;
}

static struct msi_domain_ops xilinx_msi_domain_ops = {
	.get_hwirq = xilinx_pcie_msi_get_hwirq,
	.msi_free = xilinx_pcie_msi_free,
	.msi_prepare = xilinx_pcie_msi_prepare,
};

static struct irq_chip xilinx_msi_irq_chip = {
	.name = "xdma:msi",
	.irq_mask = pci_msi_mask_irq,
	.irq_unmask = pci_msi_unmask_irq,
	.irq_compose_msi_msg = xilinx_compose_msi_msg,
	.irq_set_affinity = xilinx_msi_set_affinity,
};

static struct msi_domain_info xilinx_msi_domain_info = {
	.flags = (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		MSI_FLAG_MULTI_PCI_MSI | MSI_FLAG_PCI_MSIX),
	.ops = &xilinx_msi_domain_ops,
	.chip = &xilinx_msi_irq_chip,
	.handler = handle_simple_irq,
	.handler_name = "Xilinx MSI",
};

/**
 * xilinx_pcie_init_msi - Enable MSI support
 * @port: PCIe port information
 */
static int xilinx_pcie_init_msi(struct xilinx_pcie_port *port)
{
	struct xilinx_msi *msi = &port->msi;
	int size = BITS_TO_LONGS(XILINX_NUM_MSI_IRQS) * sizeof(long);

	mutex_init(&msi->lock);
	
	msi->msi_irq_in_use = kzalloc(size, GFP_KERNEL);
	if (!msi->msi_irq_in_use)
	        return -ENOMEM;

	msi->msi_pages = __get_free_pages(GFP_KERNEL, 0);
	msi->msi_phys = virt_to_phys((void *)msi->msi_pages);
	pcie_write(port, upper_32_bits(msi->msi_phys), XILINX_PCIE_REG_MSIBASE1);
	pcie_write(port, lower_32_bits(msi->msi_phys), XILINX_PCIE_REG_MSIBASE2);
	
	return 0;
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

	port->leg_domain = irq_domain_add_linear(pcie_intc_node, INTX_NUM,
						 &intx_domain_ops, port);
	if (!port->leg_domain) {
		dev_err(dev, "Failed to get a INTx IRQ domain\n");
		return PTR_ERR(port->leg_domain);
	}

	return xilinx_pcie_init_msi(port);
}

static void xilinx_pcie_port_free(struct xilinx_pcie_port *port)
{
	struct xilinx_pcie *pcie = port->pcie;
	struct device *dev = pcie->dev;

	list_del(&port->list);
	devm_kfree(dev, port);
}

static void xilinx_pcie_put_resources(struct xilinx_pcie *pcie)
{
	struct xilinx_pcie_port *port, *tmp;

	list_for_each_entry_safe(port, tmp, &pcie->ports, list)
		xilinx_pcie_port_free(port);
}

/**
 * xilinx_pcie_register_host - register 
 * tree-like PCIe bridges, buses and devices to host
 * @pcie: Xilinx PCIe structure
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_pcie_register_host(struct pci_host_bridge *host)
{
	struct xilinx_pcie *pcie = pci_host_bridge_priv(host);
	struct pci_bus *child;
	int err;

	host->busnr = pcie->busn.start;
	host->dev.parent = pcie->dev;
	host->ops = &xilinx_pcie_ops;
	host->sysdata = pcie;
	host->map_irq = of_irq_parse_and_map_pci;
	host->swizzle_irq = pci_common_swizzle;

	err = pci_scan_root_bus_bridge(host);
	if (err < 0)
		return err;

	pci_assign_unassigned_bus_resources(host->bus);

	list_for_each_entry(child, &host->bus->children, node)
		pcie_bus_configure_settings(child);

	pci_bus_add_devices(host->bus);

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
		dev_info(port->dev, "PCIe Link is DOWN\n");
		xilinx_pcie_port_free(port);
		return;
	}
	
	dev_info(port->dev, "PCIe Link is UP\n");

	/* Disable all interrupts */
	pcie_write(port, ~XILINX_PCIE_IDR_ALL_MASK,
		   XILINX_PCIE_REG_IMR);

	/* Clear pending interrupts */
	pcie_write(port, pcie_read(port, XILINX_PCIE_REG_IDR) &
		XILINX_PCIE_IMR_ALL_MASK, XILINX_PCIE_REG_IDR);

	/* Enable all interrupts */
	pcie_write(port, XILINX_PCIE_IMR_ALL_MASK, XILINX_PCIE_REG_IMR);
	pcie_write(port, XILINX_PCIE_IDRN_MASK, XILINX_PCIE_REG_IDRN_MASK);
	if (port->msi_mode == MSI_DECD_MODE) {
		pcie_write(port, XILINX_PCIE_IDR_ALL_MASK,
			   XILINX_PCIE_REG_MSI_LOW_MASK);
		pcie_write(port, XILINX_PCIE_IDR_ALL_MASK,
			   XILINX_PCIE_REG_MSI_HI_MASK);
	}

	/* Enable the Bridge enable bit */
	pcie_write(port, pcie_read(port, XILINX_PCIE_REG_RPSC) |
			 XILINX_PCIE_REG_RPSC_BEN,
		   XILINX_PCIE_REG_RPSC);
}

/**
 * xilinx_pcie_setup_msi_cpu_mask - Bounding MSI GIC IRQ to a specified CPU core
 * @irq: MSI GIC IRQ number
 * @cpu: Device Tree Node to parse
 *
 * Return: '0' on success and error value on failure
 */
static int xilinx_pcie_setup_msi_cpu_mask(int irq, int cpu)
{
	cpumask_var_t mask;
	int err;

	int num_cpus = num_possible_cpus();

	if ((num_cpus == 1) && cpu)
		cpu = 0;
	
	if (alloc_cpumask_var(&mask, GFP_KERNEL))
	{
		cpumask_clear(mask);
		cpumask_set_cpu(cpu, mask);
		
		err = irq_set_affinity(irq, mask);
		
		if (err)
		        pr_err("Failed to set affinity for MSI GIC IRQ %d\n", irq);
			
		free_cpumask_var(mask);
	}
	else
	        err = -EINVAL;
		
	return err;
}

/**
 * xilinx_pcie_setup_irq - Setup INTx/MSI interrupt for specified root port
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

	err = xilinx_pcie_init_irq_domain(port, node);
	if (err) {
		dev_err(dev, "failed to init PCIe IRQ domain\n");
		return err;
	}

	/* MSI FIFO mode */
	if (port->msi_mode == MSI_FIFO_MODE)
	{
		irq = platform_get_irq(pdev, port->slot);
		if (irq <= 0) {
			dev_err(dev, "Unable to find IRQ line on port %d\n", port->slot);
			return -ENXIO;
		}
		
		err = devm_request_irq(dev, irq, xilinx_pcie_intr_handler,
			       IRQF_SHARED | IRQF_NO_THREAD, "xilinx-pcie", port);
			       
		if (err) {
			dev_err(dev, "unable to request IRQ %d\n", irq);
			return err;
		}
	}
	
	/*MSI Decode mode (newly support)*/
	else if (port->msi_mode == MSI_DECD_MODE)
	{
		/*request misc irq*/
		irq = platform_get_irq(pdev, (port->slot * REQ_DECD_TOTAL + REQ_DECD_MISC));
		if (irq <= 0) {
			dev_err(dev, "Unable to find misc IRQ line\n");
			return irq;
		}
		err = devm_request_irq(dev, irq, xilinx_pcie_intr_handler,
				       IRQF_SHARED | IRQF_NO_THREAD, "xilinx-pcie", port);
		if (err) {
			dev_err(dev, "unable to request misc IRQ line of port %d\n", port->slot);
			return err;
		}
		
		/*request msi0 req*/
		irq = platform_get_irq(pdev, (port->slot * REQ_DECD_TOTAL + REQ_DECD_MSI0));
		if (irq <= 0) {
			dev_err(dev, "Unable to find msi0 IRQ line\n");
			return irq;
		}
		
		irq_set_chained_handler_and_data(irq, xilinx_pcie_msi_handler_low, port);
		err = xilinx_pcie_setup_msi_cpu_mask(irq, 0);
		if (err)
		{
			irq_set_chained_handler_and_data(irq, NULL, NULL);
			return err;
		}
		
		/*request msi1 req*/
		irq = platform_get_irq(pdev, (port->slot * REQ_DECD_TOTAL + REQ_DECD_MSI1));
		if (irq <= 0) {
			dev_err(dev, "Unable to find msi1 IRQ line\n");
			return irq;
		}
		
		irq_set_chained_handler_and_data(irq, xilinx_pcie_msi_handler_high, port);
		
		err = xilinx_pcie_setup_msi_cpu_mask(irq, 1);
		if (err)
		{
			irq_set_chained_handler_and_data(irq, NULL, NULL);
			return err;
		}
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
	int err, mode_val, val;

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	snprintf(name, sizeof(name), "rp%d", slot);
	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	port->reg_base = devm_pci_remap_cfg_resource(dev, regs);
	if (IS_ERR(port->reg_base)) {
		dev_err(dev, "failed to map port%d base\n", slot);
		return PTR_ERR(port->reg_base);
	}

	port->slot = slot;
	port->pcie = pcie;

	/*check MSI interrupt mode*/
	if (pcie->xdma_config == XDMA_ZYNQMP_PL) {
		val = pcie_read(port, XILINX_PCIE_REG_BIR);
		val = (val >> XILINX_PCIE_FIFO_SHIFT) & MSI_DECD_MODE;
		mode_val = pcie_read(port, XILINX_PCIE_REG_VSEC) &
				XILINX_PCIE_VSEC_REV_MASK;
		mode_val = mode_val >> XILINX_PCIE_VSEC_REV_SHIFT;
		if (mode_val && !val) {
			port->msi_mode = MSI_DECD_MODE;
			dev_info(dev, "Using MSI Decode mode\n");
		} else {
			port->msi_mode = MSI_FIFO_MODE;
			dev_info(dev, "Using MSI FIFO mode\n");
		}
	}

	if (pcie->xdma_config == XDMA_VERSAL_PL)
		port->msi_mode = MSI_DECD_MODE;
	
	/*setup interrupt*/
	err = xilinx_pcie_setup_irq(port, node);
	if (err)
		return err;

	INIT_LIST_HEAD(&port->list);
	list_add_tail(&port->list, &pcie->ports);

	return 0;
}

/**
 * xilinx_pcie_parse_dt - Parse Device tree and setup each root port
 * @port: PCIe root complex (RC) information
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

	if (of_device_is_compatible(node, "xlnx,xdma-host-3.00"))
		pcie->xdma_config = XDMA_ZYNQMP_PL;
	else if (of_device_is_compatible(node, "xlnx,pcie-dma-versal-2.0"))
		pcie->xdma_config = XDMA_VERSAL_PL;

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

	struct pci_host_bridge *bridge;
	int err;
	LIST_HEAD(res);
	
	struct fwnode_handle *fwnode;
	struct msi_domain_info *info;

	/*check if Device Tree Node is available*/
	if (!dev->of_node)
		return -ENODEV;

	/* Allocation of struct pci_host_bridge as a new API 
	 * to scan PCI bus */
	bridge = devm_pci_alloc_host_bridge(dev, sizeof(*pcie));
	if (!bridge)
		return -ENOMEM;

	pcie = pci_host_bridge_priv(bridge);

	pcie->dev = dev;
	platform_set_drvdata(pdev, pcie);
	INIT_LIST_HEAD(&pcie->ports);
	
	/*setup PCIe MSI domain for root bus*/
	fwnode = of_node_to_fwnode(dev->of_node);
	
	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if(!info)
	        return -ENOMEM;
		
	memcpy(info, &xilinx_msi_domain_info, sizeof(*info));
	info->chip_data = pcie;
	
	pcie->msi_domain = pci_msi_create_irq_domain(fwnode, info, NULL);
	if (!pcie->msi_domain) {
		dev_err(dev, "failed to create MSI domain");
		return -ENOMEM;
	}

	/*setup root complex and root ports by parsing device tree*/
	err = xilinx_pcie_parse_dt(pcie);
	if (err) {
		dev_err(dev, "Parsing DT failed\n");
		return err;
	}

	/*register tree-like PCIe resources to host*/
	err = xilinx_pcie_register_host(bridge);
	if (err)
		goto put_resources;

	return 0;

put_resources:
	if (!list_empty(&pcie->ports))
		xilinx_pcie_put_resources(pcie);

	return err;
}

static const struct of_device_id xilinx_pcie_of_match[] = {
	{ .compatible = "xlnx,xdma-host-3.00", },
	{ .compatible = "xlnx,pcie-dma-versal-2.0", },
	{}
};

static struct platform_driver xilinx_pcie_driver = {
	.driver = {
		.name = "xilinx-xdma-pcie",
		.of_match_table = xilinx_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = xilinx_pcie_probe,
};

builtin_platform_driver(xilinx_pcie_driver);
