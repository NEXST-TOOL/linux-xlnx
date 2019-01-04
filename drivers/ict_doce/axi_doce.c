/**
 * @file axi_doce.c
 * @date Feb. 15, 2017
 * @author Yisong Chang
 *
 * This file contains the top level functions for AXI DoCE module.
 *
 **/

// Kernel dependencies
#include <linux/module.h>           // Module init and exit macros
#include <linux/moduleparam.h>      // Module param macro
#include <linux/slab.h>             // Allocation functions
#include <linux/stat.h>             // Module parameter permission values
#include <linux/platform_device.h>  // Platform device definitions

// Local dependencies
#include "axidoce.h"                 // Internal definitions

/*----------------------------------------------------------------------------
 * Module Parameters
 *----------------------------------------------------------------------------*/

// The name to use for the character device. This is "axi_doce" by default.
static char *chrdev_name = CHRDEV_NAME;
module_param(chrdev_name, charp, S_IRUGO);

// The minor number to use for the character device. 0 by default.
static int minor_num = MINOR_NUMBER;
module_param(minor_num, int, S_IRUGO);

/*----------------------------------------------------------------------------
 * Platform Device Functions
 *----------------------------------------------------------------------------*/

static int axidoce_probe(struct platform_device *pdev)
{
    int rc;
    struct axidoce_device *axidoce_dev;

    // Allocate an AXI DoCE device structure to hold metadata about the DoCE
    axidoce_dev = kmalloc(sizeof(*axidoce_dev), GFP_KERNEL);
    if (axidoce_dev == NULL) {
        axidoce_err("Unable to allocate the AXI DoCE device structure.\n");
        return -ENOMEM;
    }
    axidoce_dev->pdev = pdev;

    // Initialize the DoCE interface
    rc = axidoce_main_init(pdev, axidoce_dev);
    if (rc < 0) {
        goto free_axidoce_dev;
    }

    // Assign the character device name, minor number, and number of devices
    axidoce_dev->chrdev_name = chrdev_name;
    axidoce_dev->minor_num = minor_num;
    axidoce_dev->num_devices = NUM_DEVICES;

    // Initialize the character device for the module.
    rc = axidoce_chrdev_init(axidoce_dev);
    if (rc < 0) {
        goto destroy_doce_dev;
    }

    // Set the private data in the device to the AXI DoCE device structure
    dev_set_drvdata(&pdev->dev, axidoce_dev);
    return 0;

destroy_doce_dev:
    axidoce_main_exit(axidoce_dev);
free_axidoce_dev:
    kfree(axidoce_dev);
    return -ENOSYS;
}

static int axidoce_remove(struct platform_device *pdev)
{
    struct axidoce_device *axidoce_dev;

    // Get the AXI DoCE device structure from the device's private data
    axidoce_dev = dev_get_drvdata(&pdev->dev);

    // Cleanup the character device structures
    axidoce_chrdev_exit(axidoce_dev);

    // Cleanup the DoCE structures
    axidoce_main_exit(axidoce_dev);

    // Free the device structure
    kfree(axidoce_dev);
    return 0;
}

static const struct of_device_id axidoce_of_match[] = {
    { .compatible = "xlnx,axidoce-chrdev" },
    {}
};

MODULE_DEVICE_TABLE(of, axidoce_of_match);

static struct platform_driver axidoce_driver = {
    .driver = {
        .name = MODULE_NAME,
        .owner = THIS_MODULE,
        .of_match_table = axidoce_of_match,
    },
    .probe = axidoce_probe,
    .remove = axidoce_remove,
};

/*----------------------------------------------------------------------------
 * Module Initialization and Exit
 *----------------------------------------------------------------------------*/

static int __init axidoce_init(void)
{
    return platform_driver_register(&axidoce_driver);
}

static void __exit axidoce_exit(void)
{
    return platform_driver_unregister(&axidoce_driver);
}

module_init(axidoce_init);
module_exit(axidoce_exit);

MODULE_AUTHOR("Yisong Chang");

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("DoCE Module for allocation of user-level physical contiguous pinned local memory"
                   "and L1/L2 cache invalidation for remote memory.");

