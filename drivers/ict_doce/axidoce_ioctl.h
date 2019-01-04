/**
 * @file axidoce_ioctl.h
 * @date Tuesday, November 24, 2015 at 09:48:17 PM EST
 * @author Brandon Perez (bmperez)
 * @author Jared Choi (jaewonch)
 *
 * This file contains the IOCTL interface definition. This is the interface from
 * userspace to the AXI DMA device to initiate DMA transactions and get
 * information about AXI DMA devices on the system.
 **/

#ifndef AXIDOCE_IOCTL_H_
#define AXIDOCE_IOCTL_H_

#include <asm/ioctl.h>              // IOCTL macros

/*----------------------------------------------------------------------------
 * IOCTL Defintions
 *----------------------------------------------------------------------------*/

// The standard name for the AXI DOCE device
#define AXIDOCE_DEV_NAME     "axidoce"

// The standard path to the AXI DOCE device
#define AXIDOCE_DEV_PATH     ("/dev/" AXIDOCE_DEV_NAME)

/*----------------------------------------------------------------------------
 * IOCTL Argument Definitions
 *----------------------------------------------------------------------------*/

//DoCE table operations: write/read/clear an entry
enum axidoce_table_op {
    AXIDOCE_WRITE,
    AXIDOCE_READ,
	AXIDOCE_CLEAR
};

//DoCE hardware engine operation
enum axidoce_op {
    AXIDOCE_START,
    AXIDOCE_RESET
};

//operations to shadow window table
struct axidoce_swt_ops {
	enum axidoce_table_op op_type;
	int entry_offset;
	unsigned long long shadow_base_addr;
	unsigned long long shadow_high_addr;
	unsigned int context_id;
};

//operations to global partitioned virtual address mapping table
struct axidoce_gpva_mt_ops {
	enum axidoce_table_op op_type;
	int context_id;
	int connection_id;
	unsigned long long buf_start_offset;
	unsigned long long buf_end_offset;
};

//operations to physical mapping table
struct axidoce_pmt_ops {
	enum axidoce_table_op op_type;
	int context_id;
	unsigned long long buf_pa_base;
};

//operations to connection table
struct axidoce_conn_tab_ops {
	enum axidoce_table_op op_type;
	int connection_id;
	unsigned long long mac_addr;
	unsigned int ip_addr;
};

//operations to start/reset DoCE
struct axidoce_ctrl {
	enum axidoce_op op_type;
};

//user-level barrier
struct axidoce_barrier {
	unsigned int context_id;
};

//user-level cache invalidate for remote memory 
struct axidoce_cache_invalid {
	void *virt_addr;
	unsigned int phy_start_addr;
	unsigned int phy_end_addr;
	unsigned int size;
};

/*----------------------------------------------------------------------------
 * IOCTL Interface
 *----------------------------------------------------------------------------*/

// The magic number used to distinguish IOCTL's for our device
#define AXIDOCE_IOCTL_MAGIC              'W'

// The number of IOCTL's implemented, used for verification
#define AXIDOCE_NUM_IOCTLS               7

/*
 * AXI DoCE shadow window table operation.
 */
#define AXIDOCE_SWT_OPS     _IOR(AXIDOCE_IOCTL_MAGIC, 0, \
                                             struct axidoce_swt_ops)

/*
 * AXI DoCE global partitioned virtual address mapping table operation.
 */
#define AXIDOCE_GPVA_MT_OPS     _IOR(AXIDOCE_IOCTL_MAGIC, 1, \
                                             struct axidoce_gpva_mt_ops)

/*
 * AXI DoCE physical mapping table operation.
 */
#define AXIDOCE_PMT_OPS     _IOR(AXIDOCE_IOCTL_MAGIC, 2, \
                                             struct axidoce_pmt_ops)

/*
 * AXI DoCE connection table operation.
 */
#define AXIDOCE_CONN_TAB_OPS     _IOR(AXIDOCE_IOCTL_MAGIC, 3, \
                                             struct axidoce_conn_tab_ops)

/*
 * AXI DoCE start/reset DoCE.
 */
#define AXIDOCE_CTRL_OPS     _IOR(AXIDOCE_IOCTL_MAGIC, 4, \
                                             struct axidoce_ctrl)

/*
 * AXI DoCE user-level barrier
 */
#define AXIDOCE_BARRIER_OPS     _IOR(AXIDOCE_IOCTL_MAGIC, 5, \
                                             struct axidoce_barrier)

/*
 * AXI DoCE cache invalidate for remote memory
 */
#define AXIDOCE_CACHE_INVALID_OPS     _IOR(AXIDOCE_IOCTL_MAGIC, 6, \
                                             struct axidoce_cache_invalid)

#endif /* AXIDOCE_IOCTL_H_ */
