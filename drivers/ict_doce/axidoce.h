/**
 * @file axidoce.h
 * @date Feb. 15, 2017
 * @author Yisong Chang
 *
 * This file contains the internal definitions and structures for AXI DoCE module
 *
 **/

#ifndef AXIDOCE_H_
#define AXIDOCE_H_

// Kernel dependencies
#include <linux/list.h>         // Linked list definitions and functions
#include <linux/kernel.h>           // Contains the definition for printk
#include <linux/device.h>           // Definitions for class and device structs
#include <linux/cdev.h>             // Definitions for character device structs
#include <linux/signal.h>           // Definition of signal numbers
#include <linux/dmaengine.h>        // Definitions for DMA structures and types
#include <linux/platform_device.h>  // Defintions for a platform device

// Local dependencies
#include "axidoce_ioctl.h"           // IOCTL argument structures

/*----------------------------------------------------------------------------
 * Module Definitions
 *----------------------------------------------------------------------------*/

#define MODULE_NAME                 "axi_doce_drv"

// Truncates the full __FILE__ path, only displaying the basename
#define __FILENAME__ \
    (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

// Convenient macros for printing out messages to the kernel log buffer
#define axidoce_err(fmt, ...) \
    printk(KERN_ERR MODULE_NAME ": %s: %s: %d: " fmt, __FILENAME__, __func__, \
           __LINE__, ## __VA_ARGS__)
#define axidoce_info(fmt, ...) \
    printk(KERN_INFO MODULE_NAME ": %s: %s: %d: " fmt, __FILENAME__, __func__, \
            __LINE__, ## __VA_ARGS__)

#define CONTEXT_ID_BASE		0x0000
#define SHADOW_ADDR_BASE	0x0008
#define SHADOW_ADDR_SHIFT	4

#define GPVA_MT_BASE		0x0030
#define GPVA_MT_ADDR_BASE	0x0040
#define GPVA_MT_STEP		0x0050
#define GPVA_MT_ADDR_SHIFT	4

#define PMT_BASE			0x00D0
#define PMT_ADDR_SHIFT		3

#define BARRIER_BASE		0x00E0

#define MAC_ADDR_BASE		0x0200
#define MAC_ADDR_SHIFT		3
#define IP_ADDR_BASE		0x0220
#define MAC_VALID			0x0230

#define CTRL_REG			0x0100

#define LOCAL_MAC_LOW		0x1010
#define LOCAL_MAC_HIGH		0x1014
#define LOCAL_IP			0x1018

#define WORD_ALIGN_SHIFT	2

#define CONTEXT_ID_VALID_MASK		(1 << 4)
#define CONNECTION_ID_VALID_MASK	(1 << 4)


// All of the meta-data needed for an axidoce device
struct axidoce_device {
    int num_devices;                // The number of devices
    unsigned int minor_num;         // The minor number of the device
    dev_t dev_num;                  // The device number of the device
    char *chrdev_name;              // The name of the character device
    struct device *device;          // Device structure for the char device
    struct class *dev_class;        // The device class for the chardevice
    struct cdev chrdev;             // The character device structure

    struct platform_device *pdev;   // The platofrm device from the device tree
    struct list_head local_mem_buf_list;   // List of allocated local memory buffer

	/*MMIO registers*/
	void __iomem *regs;
};

/*----------------------------------------------------------------------------
 * DoCE MMIO register read/write function 
 *----------------------------------------------------------------------------*/

/* Read/Write access to the registers */
#ifndef out_be32
#ifdef CONFIG_ARCH_ZYNQ
#define in_be32(offset)		__raw_readl(offset)
#define out_be32(offset, val)	__raw_writel(val, offset)
#endif
#endif

/*
 * axidoce_ior - Memory mapped DoCE registers read
 * This function returns the contents of the corresponding register.
 */
static inline u32 axidoce_ior(struct axidoce_device *lp, off_t offset)
{
	return in_be32(lp->regs + offset);
}

/*
 * axidoce_iow - Memory mapped DoCE registers write
 * This function writes the desired value into the corresponding register.
 */
static inline void axidoce_iow(struct axidoce_device *lp, off_t offset,
			       u32 value)
{
	out_be32((lp->regs + offset), value);
}

/*----------------------------------------------------------------------------
 * IOCTL main functions
 *----------------------------------------------------------------------------
 */
void axidoce_write_swt_entry(struct axidoce_device *dev, struct axidoce_swt_ops *swt_ops,
							int clear_op);
void axidoce_read_swt_entry(struct axidoce_device *dev, struct axidoce_swt_ops *swt_ops);

void axidoce_write_gpva_mt_entry(struct axidoce_device *dev, struct axidoce_gpva_mt_ops *gpva_mt_ops,
							int clear_op);
void axidoce_read_gpva_mt_entry(struct axidoce_device *dev, struct axidoce_gpva_mt_ops *gpva_mt_ops);

void axidoce_write_pmt_entry(struct axidoce_device *dev, struct axidoce_pmt_ops *pmt_ops,
							int clear_op);
void axidoce_read_pmt_entry(struct axidoce_device *dev, struct axidoce_pmt_ops *pmt_ops);

void axidoce_write_conn_tab_entry(struct axidoce_device *dev, struct axidoce_conn_tab_ops *conn_tab_ops,
							int clear_op);
void axidoce_read_conn_tab_entry(struct axidoce_device *dev, struct axidoce_conn_tab_ops *conn_tab_ops);

void axidoce_control_ops(struct axidoce_device *dev, struct axidoce_ctrl *ctrl_ops);

void axidoce_cache_invalid(struct axidoce_device *dev, struct axidoce_cache_invalid *user_cache_invld);

void axidoce_user_level_barrier(struct axidoce_device *dev, struct axidoce_barrier *user_level_barrier);

/*----------------------------------------------------------------------------
 * Character Device Definitions
 *----------------------------------------------------------------------------*/

// Default name of the character of the device
#define CHRDEV_NAME                 AXIDOCE_DEV_NAME
// Default minor number for the device
#define MINOR_NUMBER                0
// The default number of character devices for DoCE
#define NUM_DEVICES                 1

// Function prototypes
int axidoce_chrdev_init(struct axidoce_device *dev);
void axidoce_chrdev_exit(struct axidoce_device *dev);

/*----------------------------------------------------------------------------
 * DoCE Device Function Definitions
 *----------------------------------------------------------------------------*/

// Checks that the given integer is a valid notification signal for DoCE
#define VALID_NOTIFY_SIGNAL(signal) \
    (SIGRTMIN <= (signal) && (signal) <= SIGRTMAX)

// Function Prototypes
int axidoce_main_init(struct platform_device *pdev, struct axidoce_device *dev);
void axidoce_main_exit(struct axidoce_device *dev);

#endif /* AXIDOCE_H_ */
