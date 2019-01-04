/**
 * @file axidoce_main.c
 * @date Feb 15, 2017
 * @author Yisong Chang
 *
 * This module contains the main functions of AXI DoCE module.
 *
 * @bug No known bugs.
 **/

// Kernel dependencies
#include <linux/platform_device.h>  // Platform device definitions
#include <linux/device.h>           // Device definitions and functions
#include <linux/of.h>
#include <linux/of_address.h>
#include <asm/cacheflush.h>

// Local dependencies
#include "axidoce.h"                // Internal definitions
#include "axidoce_ioctl.h"          // IOCTL interface definition and types

//DoCE SWT write operation
void axidoce_write_swt_entry(struct axidoce_device *dev, struct axidoce_swt_ops *swt_ops, int clear_op)
{
	unsigned int offset = swt_ops->entry_offset;
	unsigned int val;

	//set context_id register first
	val = clear_op ? 0 : ((swt_ops->context_id) | CONTEXT_ID_VALID_MASK);
	axidoce_iow(dev, (CONTEXT_ID_BASE + (offset << WORD_ALIGN_SHIFT)), val);

	//set shadow base/high register
	val = clear_op ? 0 : (unsigned int)(swt_ops->shadow_base_addr & 0xFFFFFFFFF);
	axidoce_iow(dev, (SHADOW_ADDR_BASE + (offset << SHADOW_ADDR_SHIFT)), val);

	val = clear_op ? 0 : (unsigned int)((swt_ops->shadow_base_addr >> 32) & 0xFFFFFFFFF);
	axidoce_iow(dev, (SHADOW_ADDR_BASE + (offset << SHADOW_ADDR_SHIFT) + 0x4), val);

	val = clear_op ? 0 : (unsigned int)(swt_ops->shadow_high_addr & 0xFFFFFFFFF);
	axidoce_iow(dev, (SHADOW_ADDR_BASE + (offset << SHADOW_ADDR_SHIFT) + 0x8), val);

	val = clear_op ? 0 : (unsigned int)((swt_ops->shadow_high_addr >> 32) & 0xFFFFFFFFF);
	axidoce_iow(dev, (SHADOW_ADDR_BASE + (offset << SHADOW_ADDR_SHIFT) + 0xC), val);
}

//DoCE SWT read operation
void axidoce_read_swt_entry(struct axidoce_device *dev, struct axidoce_swt_ops *swt_ops)
{
	unsigned int offset = swt_ops->entry_offset;
	unsigned long long addr_l, addr;

	//read context_id register first
	swt_ops->context_id = axidoce_ior(dev, (CONTEXT_ID_BASE + (offset << WORD_ALIGN_SHIFT)));
#if 1	
    printk(KERN_ERR "swt: %08x %08x\n", (CONTEXT_ID_BASE + (offset << WORD_ALIGN_SHIFT)), swt_ops->context_id);
#endif

	//read shadow base/high register
	addr_l = (unsigned long long)axidoce_ior(dev, (SHADOW_ADDR_BASE + (offset << SHADOW_ADDR_SHIFT)));
	addr = (unsigned long long)axidoce_ior(dev, (SHADOW_ADDR_BASE + (offset << SHADOW_ADDR_SHIFT) + 0x4));
#if 1	
    printk(KERN_ERR "swt: %08x %08x\n", (SHADOW_ADDR_BASE + (offset << WORD_ALIGN_SHIFT)), addr_l);
    printk(KERN_ERR "swt: %08x %08x\n", (SHADOW_ADDR_BASE + (offset << WORD_ALIGN_SHIFT) + 0x4), addr);
#endif
	addr = (addr << 32) | addr_l;
	swt_ops->shadow_base_addr = addr;

	addr_l = (unsigned long long)axidoce_ior(dev, (SHADOW_ADDR_BASE + (offset << SHADOW_ADDR_SHIFT) + 0x8));
	addr = (unsigned long long)axidoce_ior(dev, (SHADOW_ADDR_BASE + (offset << SHADOW_ADDR_SHIFT) + 0xC));
#if 1	
    printk(KERN_ERR "swt: %08x %08x\n", (SHADOW_ADDR_BASE + (offset << WORD_ALIGN_SHIFT) + 0x8), addr_l);
    printk(KERN_ERR "swt: %08x %08x\n", (SHADOW_ADDR_BASE + (offset << WORD_ALIGN_SHIFT) + 0xC), addr);
#endif	
	addr = (addr << 32) | addr_l;
	swt_ops->shadow_high_addr = addr;
}

//GPVA-MT write operation
void axidoce_write_gpva_mt_entry(struct axidoce_device *dev, struct axidoce_gpva_mt_ops *gpva_mt_ops, int clear_op)
{
	unsigned int context_id = gpva_mt_ops->context_id;
	unsigned int connection_id = gpva_mt_ops->connection_id;

	unsigned int gpva_mt_conn_base = GPVA_MT_BASE + (context_id * GPVA_MT_STEP);
	unsigned int gpva_mt_buf_addr_base = GPVA_MT_ADDR_BASE + (context_id * GPVA_MT_STEP);

	unsigned int val;

	//set context_id register first
	val = clear_op ? 0 : (connection_id | CONNECTION_ID_VALID_MASK);
	axidoce_iow(dev, (gpva_mt_conn_base + (connection_id << WORD_ALIGN_SHIFT)), val);

	//set shadow base/high register
	val = clear_op ? 0 : (unsigned int)(gpva_mt_ops->buf_start_offset & 0xFFFFFFFFF);
	axidoce_iow(dev, (gpva_mt_buf_addr_base + (connection_id << GPVA_MT_ADDR_SHIFT)), val);

	val = clear_op ? 0 : (unsigned int)((gpva_mt_ops->buf_start_offset >> 32) & 0xFFFFFFFFF);
	axidoce_iow(dev, (gpva_mt_buf_addr_base + (connection_id << GPVA_MT_ADDR_SHIFT) + 0x4), val);

	val = clear_op ? 0 : (unsigned int)(gpva_mt_ops->buf_end_offset & 0xFFFFFFFFF);
	axidoce_iow(dev, (gpva_mt_buf_addr_base + (connection_id << GPVA_MT_ADDR_SHIFT) + 0x8), val);

	val = clear_op ? 0 : (unsigned int)((gpva_mt_ops->buf_end_offset >> 32) & 0xFFFFFFFFF);
	axidoce_iow(dev, (gpva_mt_buf_addr_base + (connection_id << GPVA_MT_ADDR_SHIFT) + 0xC), val);
}

//GPVA-MT read function
void axidoce_read_gpva_mt_entry(struct axidoce_device *dev, struct axidoce_gpva_mt_ops *gpva_mt_ops)
{
	unsigned int context_id = gpva_mt_ops->context_id;
	unsigned int connection_id = gpva_mt_ops->connection_id;

	unsigned int gpva_mt_conn_base = GPVA_MT_BASE + (context_id * GPVA_MT_STEP);
	unsigned int gpva_mt_buf_addr_base = GPVA_MT_ADDR_BASE + (context_id * GPVA_MT_STEP);

	unsigned long long addr_l, addr;

	//read connection_id register first
	gpva_mt_ops->connection_id = axidoce_ior(dev, (gpva_mt_conn_base + (connection_id << WORD_ALIGN_SHIFT)));
#if 1	
    printk(KERN_ERR "gpva_mt: %08x %08x\n", (gpva_mt_conn_base + (connection_id << WORD_ALIGN_SHIFT)), gpva_mt_ops->connection_id);
#endif

	//read shadow base/high register
	addr_l = (unsigned long long)axidoce_ior(dev, (gpva_mt_buf_addr_base + (connection_id << GPVA_MT_ADDR_SHIFT)));
	addr = (unsigned long long)axidoce_ior(dev, (gpva_mt_buf_addr_base + (connection_id << GPVA_MT_ADDR_SHIFT) + 0x4));
#if 1	
    printk(KERN_ERR "swt: %08x %08x\n", (gpva_mt_buf_addr_base + (connection_id << GPVA_MT_ADDR_SHIFT)), addr_l);
    printk(KERN_ERR "swt: %08x %08x\n", (gpva_mt_buf_addr_base + (connection_id << GPVA_MT_ADDR_SHIFT) + 0x4), addr);
#endif
	addr = (addr << 32) | addr_l;
	gpva_mt_ops->buf_start_offset = addr;

	addr_l = (unsigned long long)axidoce_ior(dev, (gpva_mt_buf_addr_base + (connection_id << GPVA_MT_ADDR_SHIFT) + 0x8));
	addr = (unsigned long long)axidoce_ior(dev, (gpva_mt_buf_addr_base + (connection_id << GPVA_MT_ADDR_SHIFT) + 0xC));
#if 1	
    printk(KERN_ERR "swt: %08x %08x\n", (gpva_mt_buf_addr_base + (connection_id << GPVA_MT_ADDR_SHIFT) + 0x8), addr_l);
    printk(KERN_ERR "swt: %08x %08x\n", (gpva_mt_buf_addr_base + (connection_id << GPVA_MT_ADDR_SHIFT) + 0xC), addr);
#endif
	addr = (addr << 32) | addr_l;
	gpva_mt_ops->buf_end_offset = addr;
}

//Write an entry in PMT
void axidoce_write_pmt_entry(struct axidoce_device *dev, struct axidoce_pmt_ops *pmt_ops, int clear_op)
{
	unsigned int context_id = pmt_ops->context_id;

	unsigned int pmt_base = context_id << PMT_ADDR_SHIFT;

	unsigned int val;

	val = clear_op ? 0 : (pmt_ops->buf_pa_base & 0xFFFFFFFF);
	axidoce_iow(dev, (PMT_BASE + pmt_base), val);
	
	val = clear_op ? 0 : ((pmt_ops->buf_pa_base) >> 32 & 0xFFFFFFFF);
	axidoce_iow(dev, (PMT_BASE + pmt_base + 0x4), val);
}

//read an entry in PMT
void axidoce_read_pmt_entry(struct axidoce_device *dev, struct axidoce_pmt_ops *pmt_ops)
{
	unsigned int context_id = pmt_ops->context_id;

	unsigned int pmt_base = context_id << PMT_ADDR_SHIFT;

	unsigned long long addr_l, addr;

	addr_l = axidoce_ior(dev, (PMT_BASE + pmt_base));
	addr = axidoce_ior(dev, (PMT_BASE + pmt_base + 0x4));
#if 1	
    printk(KERN_ERR "pmt: %08x %08x\n", (PMT_BASE + pmt_base), addr_l);
    printk(KERN_ERR "pmt: %08x %08x\n", (PMT_BASE + pmt_base + 0x4), addr);
#endif
	pmt_ops->buf_pa_base = (addr << 32) | addr_l;
}

//Write an entry in Connection Table
void axidoce_write_conn_tab_entry(struct axidoce_device *dev, struct axidoce_conn_tab_ops *conn_tab_ops, int clear_op)
{
	unsigned int connection_id = conn_tab_ops->connection_id;

	unsigned int conn_tab_base = connection_id << MAC_ADDR_SHIFT;

	unsigned int valid_array;
	unsigned int val;

	//MAC address low part
	val = clear_op ? 0 : (conn_tab_ops->mac_addr) & 0xFFFFFFFF;
	axidoce_iow(dev, (MAC_ADDR_BASE + conn_tab_base), val);

	//MAC address high part
	val = clear_op ? 0 : ((conn_tab_ops->mac_addr) >> 32 & 0xFFFFFFFF);
	axidoce_iow(dev, (MAC_ADDR_BASE + conn_tab_base + 0x4), val); 

	//IP address
	val = clear_op ? 0 : conn_tab_ops->ip_addr;
	axidoce_iow(dev, (IP_ADDR_BASE + (connection_id << WORD_ALIGN_SHIFT)), val); 

	//connection valid register
	valid_array = axidoce_ior(dev, MAC_VALID);
	valid_array |= (1 << connection_id);
	val = clear_op ? 0 : valid_array;

	axidoce_iow(dev, MAC_VALID, val);
}

//Read an entry in Connection Table
void axidoce_read_conn_tab_entry(struct axidoce_device *dev, struct axidoce_conn_tab_ops *conn_tab_ops)
{
	unsigned int connection_id = conn_tab_ops->connection_id;

	unsigned int conn_tab_base = connection_id << MAC_ADDR_SHIFT;

	unsigned long long addr_l, addr;

	unsigned int val;

	addr_l = axidoce_ior(dev, (MAC_ADDR_BASE + conn_tab_base));
	addr = axidoce_ior(dev, (MAC_ADDR_BASE + conn_tab_base + 0x4));
#if 1
    printk(KERN_ERR "conn_tab: %08x %08x\n", (MAC_ADDR_BASE + conn_tab_base), addr_l);
    printk(KERN_ERR "conn_tab: %08x %08x\n", (MAC_ADDR_BASE + conn_tab_base + 0x4), addr);

	val = axidoce_ior(dev, MAC_VALID);
    printk(KERN_ERR "conn_tab: %08x %08x\n", MAC_VALID, val);
#endif
	conn_tab_ops->mac_addr = (addr << 32) | addr_l;

	conn_tab_ops->ip_addr = axidoce_ior(dev, (IP_ADDR_BASE + (connection_id << WORD_ALIGN_SHIFT)));
}

//DoCE control register write
void axidoce_control_ops(struct axidoce_device *dev, struct axidoce_ctrl *ctrl_ops)
{
	switch(ctrl_ops->op_type)
	{
		case AXIDOCE_START:
			axidoce_iow(dev, CTRL_REG, 0x2);
			break;

		case AXIDOCE_RESET:
			axidoce_iow(dev, CTRL_REG, 0x1);
			break;

		default: 
			break;
	}
}

//user-level cache invalidate interface called via IOCTL
void axidoce_cache_invalid(struct axidoce_device *dev, struct axidoce_cache_invalid *user_cache_invld)
{
	size_t	size;
	phys_addr_t	start, end;

	if(user_cache_invld == NULL)
	{
		printk(KERN_ERR "empty axidoce_cache_invalid struct pointer\n");
		return;
	}

	size = (size_t)user_cache_invld->size;
	start = (phys_addr_t)user_cache_invld->phy_start_addr;
	end = (phys_addr_t)user_cache_invld->phy_end_addr;

	//invalidate L1 D-Cache first
	__cpuc_flush_dcache_area(user_cache_invld->virt_addr, size);

	//then L2 cache
	outer_inv_range(start, end);
}

//user-level cache invalidate interface called via IOCTL
void axidoce_user_level_barrier(struct axidoce_device *dev, struct axidoce_barrier *user_level_barrier)
{
	unsigned int context = user_level_barrier->context_id;

	unsigned int barrier_reg_offset = BARRIER_BASE + (context << WORD_ALIGN_SHIFT);

	//send barrier packets to other nodes first
	axidoce_iow(dev, barrier_reg_offset, 0x1);

	//polling other nodes reach barrier as well
	//exits until AXI read returns 1
	while(!axidoce_ior(dev, barrier_reg_offset));
}

//Set DoCE local MAC address
static void set_mac_address(struct axidoce_device *dev, unsigned char *address)
{
	unsigned int val = 0;

	axidoce_iow(dev, LOCAL_MAC_LOW,
			(address[0]) | 
			(address[1] << 8) |
			(address[2] << 16) |
			(address[3] << 24));

	axidoce_iow(dev, LOCAL_MAC_HIGH,
			(val | (address[4]) | (address[5] << 8)));

#if 1
	val = axidoce_ior(dev, LOCAL_MAC_LOW);
	printk(KERN_ERR "mac_low %08x\n", val);

	val = axidoce_ior(dev, LOCAL_MAC_HIGH);
	printk(KERN_ERR "mac_high %08x\n", val);
#endif
}

int axidoce_main_init(struct platform_device *pdev, struct axidoce_device *dev)
{
	struct resource doce_res;
	struct axidoce_ctrl ctrl_ops; 

	int ret;

	unsigned char mac_addr[6];

	// Map DoCE device registers
	of_address_to_resource(pdev->dev.of_node, 0, &doce_res);
	printk(KERN_ERR "%08x\n", doce_res.start);
	printk(KERN_ERR "%08x\n", doce_res.end);
	printk(KERN_ERR "%08x\n", doce_res.flags);

	dev->regs = devm_ioremap_resource(&pdev->dev, &doce_res);

	if(!dev->regs) {
		axidoce_err("could not map AXI DoCE regs.\n");
		return -ENOMEM;
	}

    printk(KERN_ERR "ioremap finished");
	printk(KERN_ERR "dev->regs: %08x\n", dev->regs);

	// Get DoCE MAC/IP address from device tree and set MMIO register
	ret = of_property_read_u8_array(pdev->dev.of_node, "local-mac-address", mac_addr, 6);
	if (ret) {
		axidoce_err("could not find MAC address\n");
		return ret;
	}

	printk(KERN_ERR "%02x %02x %02x %02x %02x %02x\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

	set_mac_address(dev, mac_addr);

	//startup DoCE
	memset(&ctrl_ops, 0 ,sizeof(ctrl_ops));
	axidoce_control_ops(dev, &ctrl_ops);

    return 0;
}

void axidoce_main_exit(struct axidoce_device *dev)
{
    return;
}
