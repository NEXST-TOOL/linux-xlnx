/**
 * @file axidma_chrdev.c
 * @date Saturday, November 14, 2015 at 12:03:13 PM EST
 * @author Brandon Perez (bmperez)
 * @author Jared Choi (jaewonch)
 *
 * This file contains the implementation of the character device for the AXI DMA
 * module.
 *
 * @bug No known bugs.
 **/

// Kernel dependencies
#include <linux/list.h>         // Linked list definitions and functions
#include <linux/sched.h>        // `Current` global variable for current task
#include <linux/device.h>       // Device and class creation functions
#include <linux/cdev.h>         // Character device functions
#include <linux/ioctl.h>        // IOCTL macros and definitions
#include <linux/fs.h>           // File operations and file types
#include <linux/mm.h>           // Memory types and remapping functions
#include <asm/uaccess.h>        // Userspace memory access functions
#include <linux/slab.h>         // Kernel allocation functions
#include <linux/errno.h>        // Linux error codes

#include <linux/dma-buf.h>      // DMA shared buffers interface
#include <linux/scatterlist.h>  // Scatter-gather table definitions

// Local dependencies
#include "axidoce.h"             // Local definitions
#include "axidoce_ioctl.h"       // IOCTL interface for the device

static struct axidoce_device *axidoce_dev;	

struct axidoce_buf_allocation {
	size_t				size;
	void				*user_addr;
	void				*kern_addr;
	dma_addr_t			dma_addr;
	struct list_head	list;
};

/*----------------------------------------------------------------------------
 * Internal Definitions
 *----------------------------------------------------------------------------*/

static void axidoce_vma_close(struct vm_area_struct *vma)
{
    struct axidoce_buf_allocation *doce_alloc;

    // Get the AXI DMA allocation data and free the DMA buffer
    doce_alloc = vma->vm_private_data;
    dma_free_coherent(NULL, doce_alloc->size, doce_alloc->kern_addr, doce_alloc->dma_addr);

    // Remove the allocation from the list, and free the structure
    list_del(&doce_alloc->list);
    kfree(doce_alloc);

    return;
}

// The VMA operations for the AXI DMA device
static const struct vm_operations_struct axidoce_vm_ops = {
    .close = axidoce_vma_close,
};

/*----------------------------------------------------------------------------
 * File Operations
 *----------------------------------------------------------------------------*/

static int axidoce_open(struct inode *inode, struct file *file)
{
    // Only the root user can open this device, and it must be exclusive
    if (!capable(CAP_SYS_ADMIN)) {
        axidoce_err("Only root can open this device.");
        return -EACCES;
    } else if (!(file->f_flags & O_EXCL)) {
        axidoce_err("O_EXCL must be specified for open()\n");
        return -EINVAL;
    }

    // Place the axidoce structure in the private data of the file
    file->private_data = (void *)axidoce_dev;
    return 0;
}

static int axidoce_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

static int axidoce_mmap(struct file *file, struct vm_area_struct *vma)
{
    int rc;
    struct axidoce_device *dev;
    struct axidoce_buf_allocation *doce_alloc;

    // Get the axidoce device structure
    dev = file->private_data;

    // Allocate a structure to store data about the DMA mapping
    doce_alloc = kmalloc(sizeof(*doce_alloc), GFP_KERNEL);
    if (doce_alloc == NULL) {
        axidoce_err("Unable to allocate VMA data structure.");
        rc = -ENOMEM;
        goto ret;
    }

    // Set the user virtual address and the size
    doce_alloc->size = vma->vm_end - vma->vm_start;
    doce_alloc->user_addr = (void *)vma->vm_start;

    // Allocate the requested region a contiguous, cached and pinned region for local usage
    /*doce_alloc->kern_addr = dma_alloc_doce_buf(NULL, doce_alloc->size,
                                              &doce_alloc->dma_addr, GFP_KERNEL);*/
    if (doce_alloc->kern_addr == NULL) {
        axidoce_err("Unable to allocate contiguous DMA memory region of size "
                   "%zu.\n", doce_alloc->size);
        axidoce_err("Please make sure that you specified cma=<size> on the "
                   "kernel command line, and the size is large enough.\n");
        rc = -ENOMEM;
        goto free_vma_data;
    }

    // Map the region into userspace
    /*rc = dma_mmap_doce_buf(NULL, vma, doce_alloc->kern_addr, doce_alloc->dma_addr,
                           doce_alloc->size);*/
    if (rc < 0) {
        axidoce_err("Unable to remap address %p to userspace address %p, size "
                   "%zu.\n", doce_alloc->kern_addr, doce_alloc->user_addr,
                   doce_alloc->size);
        goto free_dma_region;
    }

    /* Override the VMA close with our call, so that we can free the DMA region
     * when the memory region is closed. Pass in the data to do so. */
    vma->vm_ops = &axidoce_vm_ops;
    vma->vm_private_data = doce_alloc;

    // Add the allocation to the driver's list of DMA buffers
    list_add(&doce_alloc->list, &dev->local_mem_buf_list);
    return 0;

free_dma_region:
    dma_free_coherent(NULL, doce_alloc->size, doce_alloc->kern_addr,
                      doce_alloc->dma_addr);
free_vma_data:
    kfree(doce_alloc);
ret:
    return rc;
}

/* Verifies that the pointer can be read and/or written to with the given size.
 * The user specifies the mode, either readonly, or not (read-write). */
static bool axidoce_access_ok(const void __user *arg, size_t size, bool readonly)
{
    // Note that VERIFY_WRITE implies VERIFY_WRITE, so read-write is handled
    if (!readonly && !access_ok(VERIFY_WRITE, arg, size)) {
        axidoce_err("Argument address %p, size %zu cannot be written to.\n",
                   arg, size);
        return false;
    } else if (!access_ok(VERIFY_READ, arg, size)) {
        axidoce_err("Argument address %p, size %zu cannot be read from.\n",
                   arg, size);
        return false;
    }

    return true;
}

static long axidoce_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long rc;
	long user_space_reply = 0; 
    void *__user arg_ptr;
    struct axidoce_device *dev;

    struct axidoce_swt_ops swt_ops;
    struct axidoce_gpva_mt_ops gpva_mt_ops;
    struct axidoce_pmt_ops pmt_ops;
    struct axidoce_conn_tab_ops conn_tab_ops;
	struct axidoce_ctrl doce_ctrl;
	struct axidoce_barrier user_barrier;
	struct axidoce_cache_invalid user_cache_invld;

    // Coerce the arguement as a userspace pointer
    arg_ptr = (void __user *)arg;

    // Verify that this IOCTL is intended for our device, and is in range
    if (_IOC_TYPE(cmd) != AXIDOCE_IOCTL_MAGIC) {
        axidoce_err("IOCTL command magic number does not match.\n");
        return -ENOTTY;
    } else if (_IOC_NR(cmd) >= AXIDOCE_NUM_IOCTLS) {
        axidoce_err("IOCTL command is out of range for this device.\n");
        return -ENOTTY;
    }

    // Verify the input argument
    if ((_IOC_DIR(cmd) & _IOC_READ)) {
        if (!axidoce_access_ok(arg_ptr, _IOC_SIZE(cmd), false)) {
            return -EFAULT;
        }
    } else if (_IOC_DIR(cmd) & _IOC_WRITE) {
        if (!axidoce_access_ok(arg_ptr, _IOC_SIZE(cmd), true)) {
            return -EFAULT;
        }
    }

    // Get the axidoce device from the file
    dev = file->private_data;

    // Perform the specified command
    switch (cmd) {
        case AXIDOCE_SWT_OPS:
            if (copy_from_user(&swt_ops, arg_ptr, sizeof(swt_ops)) != 0) {
                axidoce_err("Unable to copy SWT operation command structure from userspace.\n");
                return -EFAULT;
            }

			switch (swt_ops.op_type)
			{
				case AXIDOCE_WRITE:
					axidoce_write_swt_entry(dev, &swt_ops, 0);
					break;

				case AXIDOCE_READ:
					axidoce_read_swt_entry(dev, &swt_ops);
					user_space_reply = 1;
					break;

				case AXIDOCE_CLEAR:
					axidoce_write_swt_entry(dev, &swt_ops, 1);
					break;
			}
		
			if (user_space_reply)
			{
				if (copy_to_user(arg_ptr, &swt_ops, sizeof(swt_ops)) != 0) {
					axidoce_err("Unable to copy SWT read result to userspace\n");
					return -EFAULT;
				}
				user_space_reply = 0;
            }
            rc = 0;
            break;

        case AXIDOCE_GPVA_MT_OPS:
            if (copy_from_user(&gpva_mt_ops, arg_ptr, sizeof(gpva_mt_ops)) != 0) {
                axidoce_err("Unable to copy GPVA-MT operation command structure from userspace.\n");
                return -EFAULT;
            }

			switch (gpva_mt_ops.op_type)
			{
				case AXIDOCE_WRITE:
					axidoce_write_gpva_mt_entry(dev, &gpva_mt_ops, 0);
					break;

				case AXIDOCE_READ:
					axidoce_read_gpva_mt_entry(dev, &gpva_mt_ops);
					user_space_reply = 1;
					break;

				case AXIDOCE_CLEAR:
					axidoce_write_gpva_mt_entry(dev, &gpva_mt_ops, 1);
					break;
			}
		
			if (user_space_reply)
			{
				if (copy_to_user(arg_ptr, &gpva_mt_ops, sizeof(gpva_mt_ops)) != 0) {
					axidoce_err("Unable to copy GPVA-MT read result to userspace\n");
					return -EFAULT;
				}
				user_space_reply = 0;
            }
            rc = 0;
            break;

        case AXIDOCE_PMT_OPS:
            if (copy_from_user(&pmt_ops, arg_ptr, sizeof(pmt_ops)) != 0) {
                axidoce_err("Unable to copy PMT operation command structure from userspace.\n");
                return -EFAULT;
            }

			switch (pmt_ops.op_type)
			{
				case AXIDOCE_WRITE:
					axidoce_write_pmt_entry(dev, &pmt_ops, 0);
					break;

				case AXIDOCE_READ:
					axidoce_read_pmt_entry(dev, &pmt_ops);
					user_space_reply = 1;
					break;

				case AXIDOCE_CLEAR:
					axidoce_write_pmt_entry(dev, &pmt_ops, 1);
					break;
			}
		
			if (user_space_reply)
			{
				if (copy_to_user(arg_ptr, &pmt_ops, sizeof(pmt_ops)) != 0) {
					axidoce_err("Unable to copy PMT read result to userspace\n");
					return -EFAULT;
				}
				user_space_reply = 0;
            }
            rc = 0;
            break;

        case AXIDOCE_CONN_TAB_OPS:
            if (copy_from_user(&conn_tab_ops, arg_ptr, sizeof(conn_tab_ops)) != 0) {
                axidoce_err("Unable to copy Connection Table operation command structure from userspace.\n");
                return -EFAULT;
            }

			switch (conn_tab_ops.op_type)
			{
				case AXIDOCE_WRITE:
					axidoce_write_conn_tab_entry(dev, &conn_tab_ops, 0);
					break;

				case AXIDOCE_READ:
					axidoce_read_conn_tab_entry(dev, &conn_tab_ops);
					user_space_reply = 1;
					break;

				case AXIDOCE_CLEAR:
					axidoce_write_conn_tab_entry(dev, &conn_tab_ops, 1);
					break;
			}
		
			if (user_space_reply)
			{
				if (copy_to_user(arg_ptr, &conn_tab_ops, sizeof(conn_tab_ops)) != 0) {
					axidoce_err("Unable to copy Connection Table read result to userspace\n");
					return -EFAULT;
				}
				user_space_reply = 0;
            }
            rc = 0;
            break;

		case AXIDOCE_CTRL_OPS:
            if (copy_from_user(&doce_ctrl, arg_ptr, sizeof(doce_ctrl)) != 0) {
                axidoce_err("Unable to copy DoCE start/reset operation command structure from userspace.\n");
                return -EFAULT;
            }
			axidoce_control_ops(dev, &doce_ctrl);
			break;

		case AXIDOCE_BARRIER_OPS:
            if (copy_from_user(&user_barrier, arg_ptr, sizeof(user_barrier)) != 0) {
                axidoce_err("Unable to copy DoCE user-level barrier operation command structure from userspace.\n");
                return -EFAULT;
            }
			axidoce_user_level_barrier(dev, &user_barrier);
			break;

        case AXIDOCE_CACHE_INVALID_OPS:
            if (copy_from_user(&user_cache_invld, arg_ptr, sizeof(user_cache_invld)) != 0) {
                axidoce_err("Unable to copy DoCE cache invalidate operation command structure from userspace.\n");
                return -EFAULT;
            }
			axidoce_cache_invalid(dev, &user_cache_invld);
			break;

        // Invalid command (already handled in preamble)
        default:
            return -ENOTTY;
    }

    return rc;
}

// The file operations for the AXI DMA device
static const struct file_operations axidoce_fops = {
    .owner = THIS_MODULE,
    .open = axidoce_open,
    .release = axidoce_release,
    .mmap = axidoce_mmap,
    .unlocked_ioctl = axidoce_ioctl,
};

/*----------------------------------------------------------------------------
 * Initialization and Cleanup
 *----------------------------------------------------------------------------*/

int axidoce_chrdev_init(struct axidoce_device *dev)
{
    int rc;

    // Store a global pointer to the axidma device
    axidoce_dev = dev;

    // Allocate a major and minor number region for the character device
    rc = alloc_chrdev_region(&dev->dev_num, dev->minor_num, dev->num_devices,
                             dev->chrdev_name);
    if (rc < 0) {
        axidoce_err("Unable to allocate character device region.\n");
        goto ret;
    }

    // Create a device class for our device
    dev->dev_class = class_create(THIS_MODULE, dev->chrdev_name);
    if (IS_ERR(dev->dev_class)) {
        axidoce_err("Unable to create a device class.\n");
        rc = PTR_ERR(dev->dev_class);
        goto free_chrdev_region;
    }

    /* Create a device for our module. This will create a file on the
     * filesystem, under "/dev/dev->chrdev_name". */
    dev->device = device_create(dev->dev_class, NULL, dev->dev_num, NULL,
                                dev->chrdev_name);
    if (IS_ERR(dev->device)) {
        axidoce_err("Unable to create a device.\n");
        rc = PTR_ERR(dev->device);
        goto class_cleanup;
    }

    // Register our character device with the kernel
    cdev_init(&dev->chrdev, &axidoce_fops);
    rc = cdev_add(&dev->chrdev, dev->dev_num, dev->num_devices);
    if (rc < 0) {
        axidoce_err("Unable to add a character device.\n");
        goto device_cleanup;
    }

    // Initialize the list for DMA mmap'ed allocations
    INIT_LIST_HEAD(&dev->local_mem_buf_list);

    return 0;

device_cleanup:
    device_destroy(dev->dev_class, dev->dev_num);
class_cleanup:
    class_destroy(dev->dev_class);
free_chrdev_region:
    unregister_chrdev_region(dev->dev_num, dev->num_devices);
ret:
    return rc;
}

void axidoce_chrdev_exit(struct axidoce_device *dev)
{
    // Cleanup all related character device structures
    cdev_del(&dev->chrdev);
    device_destroy(dev->dev_class, dev->dev_num);
    class_destroy(dev->dev_class);
    unregister_chrdev_region(dev->dev_num, dev->num_devices);

    return;
}
