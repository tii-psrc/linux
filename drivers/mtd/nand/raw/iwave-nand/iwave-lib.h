/*
 * iWave NAND Flash Controller Driver
 * based on ARM PL353 SMC driver
 * iwave systems technologies pvt. ltd., Bangalore
 * Copyright (C) 2023 iWave Systems Technologies Pvt Ltd.
 */

#ifndef __IWAVE_LIB_H
#define __IWAVE_LIB_H

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/amba/bus.h>
#include <linux/mtd/rawnand.h>


#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

/* Register definitions */
#define IW_NAND_MEMC_STATUS_OFFS              0x8     /* Controller status reg, RO */
#define IW_NAND_CFG_CLR_OFFS                  0x4     /* Clear config reg, WO */
#define IW_NAND_DIRECT_CMD_OFFS               0x10    /* Direct command reg, WO */
#define IW_NAND_SET_CYCLES_OFFS               0x14    /* Set cycles register, WO */
#define IW_NAND_SET_OPMODE_OFFS               0x18    /* Set opmode register, WO */
#define IW_NAND_ECC_ERR		              0x300   /* ECC 1 bit/2 bit error detection register RO */
#define IW_NAND_ECC_EL_DL_OFFS                0x304   /* ECC enable/dsable register, WO */
#define IW_NAND_ADDR_SIZE_DATA                0x308   /* Data size register, WO */
#define IW_NAND_TIMING_SYNC_ASYNC             0x30C   /* Timing mode and sysnc/async detection register, WO */
#define IW_NAND_ADDR_SIZE_PAGE_OOB            0x310   /* Page size and OOB size register, WO */
#define IW_NAND_ADDR_TIMEOUT_ERROR            0x314   /* Address timeout error status registor, WOC */ 
#define IW_NAND_ECC_STATUS_OFFS               0x400   /* ECC status register */
#define IW_NAND_ECC_MEMCFG_OFFS               0x404   /* ECC mem config reg */
#define IW_NAND_ECC_MEMCMD1_OFFS              0x408   /* ECC mem cmd1 reg */
#define IW_NAND_ECC_MEMCMD2_OFFS              0x40C   /* ECC mem cmd2 reg */
#define IW_NAND_ECC_VALUE0_OFFS               0x418   /* ECC value 0 reg */

/* Controller address timeout register constants */
#define IW_NAND_ADDR_TIMEOUT_ERROR_CLR	      0x1

/* Controller status register specific constants */
#define IW_NAND_MEMC_STATUS_RAW_INT_1_SHIFT   6

/* Clear configuration register specific constants */
#define IW_NAND_CFG_CLR_INT_CLR_1     	      0x10
#define IW_NAND_CFG_CLR_ECC_INT_DIS_1 	      0x40
#define IW_NAND_CFG_CLR_INT_DIS_1     	      0x2
#define IW_NAND_CFG_CLR_DEFAULT_MASK  	      (IW_NAND_CFG_CLR_INT_CLR_1 | \
		IW_NAND_CFG_CLR_ECC_INT_DIS_1 | \
		IW_NAND_CFG_CLR_INT_DIS_1)

/* Set cycles register specific constants */
#define IW_NAND_SET_CYCLES_T0_MASK    	      0xF
#define IW_NAND_SET_CYCLES_T0_SHIFT   	      0
#define IW_NAND_SET_CYCLES_T1_MASK    	      0xF
#define IW_NAND_SET_CYCLES_T1_SHIFT   	      4
#define IW_NAND_SET_CYCLES_T2_MASK    	      0x7
#define IW_NAND_SET_CYCLES_T2_SHIFT   	      8
#define IW_NAND_SET_CYCLES_T3_MASK    	      0x7
#define IW_NAND_SET_CYCLES_T3_SHIFT   	      11
#define IW_NAND_SET_CYCLES_T4_MASK    	      0x7
#define IW_NAND_SET_CYCLES_T4_SHIFT   	      14
#define IW_NAND_SET_CYCLES_T5_MASK    	      0x7
#define IW_NAND_SET_CYCLES_T5_SHIFT   	      17
#define IW_NAND_SET_CYCLES_T6_MASK    	      0xF
#define IW_NAND_SET_CYCLES_T6_SHIFT   	      20

/* ECC status register specific constants */
#define IW_NAND_ECC_STATUS_BUSY      	      (1 << 6)
#define IW_NAND_ECC_REG_SIZE_OFFS    	       4

/* ECC memory config register specific constants */
#define IW_NAND_ECC_MEMCFG_MODE_MASK  	      0xC
#define IW_NAND_ECC_MEMCFG_MODE_SHIFT 	      2
#define IW_NAND_ECC_MEMCFG_PGSIZE_MASK        0xC

#define IW_NAND_DC_UPT_NAND_REGS              ((4 << 23) |    /* CS: NAND chip */ \
		(2 << 21))     /* UpdateRegs operation */

#define IW_NAND_ECC_CMD1        	      ((0x80)       | /* Write command */ \
		(0 << 8)      | /* Read command */ \
		(0x30 << 16)  | /* Read End command */ \
		(1 << 24))     /* Read End command calid */

#define IW_NAND_ECC_CMD2        	      ((0x85)       | /* Write col change cmd */ \
		(5 << 8)      | /* Read col change cmd */ \
		(0xE0 << 16)  | /* Read col change end cmd */ \
		(1 << 24)) /* Read col change end cmd valid */
#define IW_NAND_ECC_BUSY_TIMEOUT     	      (1 * HZ)

#define IW_NAND_ECC_BUSY_TIMEOUT     	      	(1 * HZ)

/* NAND flash driver defines */
#define IW_NAND_ECC_SIZE		      512	/* Size of data for ECC operation */

/* AXI Address definitions */
#define START_CMD_SHIFT			      3
#define END_CMD_SHIFT			      11
#define END_CMD_VALID_SHIFT		      20
#define ADDR_CYCLES_SHIFT		      21
#define CLEAR_CS_SHIFT			      21
#define ECC_LAST_SHIFT			      10
#define COMMAND_PHASE			      (0 << 19)
#define IWAVE_WRITE_OOB_MASK		      (1 << 1)
#define DATA_PHASE			      BIT(19)
#define GET_ADDR(pos, val)		      (((val) & 0xFF) << (8 * (pos)))

#define IW_NAND_ECC_LAST		      BIT(ECC_LAST_SHIFT)	/* Set ECC_Last */
#define IW_NAND_CLEAR_CS		      BIT(CLEAR_CS_SHIFT)	/* Clear chip select */

#define IW_NAND_ECC_BUSY_TIMEOUT	      (1 * HZ)
#define IW_NAND_DEV_BUSY_TIMEOUT	      (1 * HZ)
#define IW_NAND_ERROR_TIMEOUT	 	      (1 * HZ)
#define IW_NAND_LAST_TRANSFER_LENGTH	      4
#define IW_NAND_ECC_VALID_SHIFT	              24
#define IW_NAND_ECC_VALID_MASK	              0x40
#define IW_ECC_BITS_BYTEOFF_MASK	      0x1FF
#define IW_ECC_BITS_BITOFF_MASK  	      0x7
#define IW_ECC_BIT_MASK		              0xFFF
#define IW_TREA_MAX_VALUE		      1
#define IW_MAX_ECC_CHUNKS		      4
#define IW_MAX_ECC_BYTES		      3
#define IW_MAX_CHUNK_SIZE		      8640

/* SMC virtual register base */

struct iwave_nand_controller {
	struct nand_controller controller;
	struct iwave_nand_chip *inand_chip;
	struct list_head chips;
	struct device *dev;
	void __iomem *nand_reg;
	void __iomem *nand_data;
	void __iomem *regs;
	struct resource *flash_reg;
	struct resource *flash_data;
	u32 dataphase_addrflags;
	u8 addr_cycles;
	ulong mclk_rate;
	u32 buswidth;
	struct proc_dir_entry *proc_dir;
	int poll_mode;
	spinlock_t lock;
};

struct iwave_nand_chip {
	struct list_head node;
	struct nand_chip chip;
	int csnum;
	int lun;
};

static inline struct iwave_nand_controller *
to_iwave_nand(struct nand_controller *ctrl)
{
	return container_of(ctrl, struct iwave_nand_controller, controller);
}


static inline struct iwave_nand_chip *
to_inand(struct nand_chip *nand)
{
	return container_of(nand, struct iwave_nand_chip, chip);
}
//void __iomem *iwave_base_reg;

struct iwave_nand_reg
{
	void __iomem *nand_reg;
	void __iomem *nand_data;
};

enum iwave_smc_ecc_mode {
	IW_NAND_ECCMODE_BYPASS = 0,
	IW_NAND_ECCMODE_APB = 1,
	IW_NAND_ECCMODE_MEM = 2
};

enum iwave_smc_mem_width {
	IW_NAND_MEM_WIDTH_8 = 0,
	IW_NAND_MEM_WIDTH_16 = 1
};

enum poll_mode {
	POLL_MODE_UNKNOWN = -1,
	POLL_MODE_SCHEDULED = 0,
	POLL_MODE_BUSY = 1,
};

u32 iwave_smc_get_ecc_val(struct iwave_nand_controller *xnfc, int ecc_reg);
bool iwave_smc_ecc_is_busy(struct iwave_nand_controller *xnfc);
int iwave_smc_get_nand_int_status_raw(struct iwave_nand_controller *xnfc, int poll_mode, unsigned long delay_us);
int iwave_read_error_reg(struct iwave_nand_controller *xnfc);
int iwave_read_error_reg(struct iwave_nand_controller *xnfc);
void iwave_smc_clr_nand_int(struct iwave_nand_controller *xnfc);
int iwave_smc_set_ecc_mode(struct iwave_nand_controller *xnfc, enum iwave_smc_ecc_mode mode);
int iwave_smc_set_ecc_pg_size(struct iwave_nand_controller *xnfc, unsigned int pg_sz);
int iwave_smc_set_buswidth(struct iwave_nand_controller *xnfc, unsigned int bw);
void iwave_smc_set_cycles(struct iwave_nand_controller *xnfc, u32 timings[]);
void iwave_nand_init_nand_interface(struct iwave_nand_controller *xnfc);
//void iwave_set_base_address(struct nand_chip *chip);
#endif /* __IWAVE_H_ */
