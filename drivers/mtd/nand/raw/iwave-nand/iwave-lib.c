// SPDX-License-Identifier: GPL-2.0
/*
 *  iWave NAND flash controller driver
 *
 * Copyright (C) 2023 iWave Systems Technologies Pvt Ltd.
 *
 */

#include <linux/iopoll.h>
#include "iwave-lib.h"


/**
 * iwave_smc_set_buswidth - Set memory buswidth
 * @bw: Memory buswidth (8 | 16)
 * Return: 0 on success or negative errno.
 */

int iwave_smc_set_buswidth(struct iwave_nand_controller *xnfc, unsigned int bw)
{
	if (bw != IW_NAND_MEM_WIDTH_8  && bw != IW_NAND_MEM_WIDTH_16)
		return -EINVAL;

        writel(bw, xnfc->regs + IW_NAND_SET_OPMODE_OFFS);
        writel(IW_NAND_DC_UPT_NAND_REGS, xnfc->regs + IW_NAND_DIRECT_CMD_OFFS);
        return 0;
}
EXPORT_SYMBOL_GPL(iwave_smc_set_buswidth);


/**
 * iwave_smc_set_ecc_mode - Set SMC ECC mode
 * @mode: ECC mode (BYPASS, APB, MEM)
 * Return: 0 on success or negative errno.
 */

int iwave_smc_set_ecc_mode(struct iwave_nand_controller *xnfc,
		enum iwave_smc_ecc_mode mode)
{
        u32 reg;
        int ret = 0;

	switch (mode) {
		case IW_NAND_ECCMODE_BYPASS:
		case IW_NAND_ECCMODE_APB:
		case IW_NAND_ECCMODE_MEM:

			reg = readl(xnfc->regs + IW_NAND_ECC_MEMCFG_OFFS);
			reg &= ~IW_NAND_ECC_MEMCFG_MODE_MASK;
			reg |= mode << IW_NAND_ECC_MEMCFG_MODE_SHIFT;
			writel(reg, xnfc->regs + IW_NAND_ECC_MEMCFG_OFFS);

			break;
		default:
			ret = -EINVAL;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(iwave_smc_set_ecc_mode);


/**
 * iwave_smc_clr_nand_int - Clear NAND interrupt
 */

void iwave_smc_clr_nand_int(struct iwave_nand_controller *xnfc)
{
	writel(IW_NAND_CFG_CLR_INT_CLR_1, xnfc->regs + IW_NAND_CFG_CLR_OFFS);
}
EXPORT_SYMBOL_GPL(iwave_smc_clr_nand_int);


/**
 * iwave_smc_set_cycles - Set memory timing parameters
 * @timings: NAND controller timing parameters
 *
 * Sets NAND chip specific timing parameters.
 */
void iwave_smc_set_cycles(struct iwave_nand_controller *xnfc, u32 timings[])
{
	/*
	 * Set write pulse timing. This one is easy to extract:
	 *
	 * NWE_PULSE = tWP
	 */

	timings[0] &= IW_NAND_SET_CYCLES_T0_MASK;
	timings[1] = (timings[1] & IW_NAND_SET_CYCLES_T1_MASK) <<
		IW_NAND_SET_CYCLES_T1_SHIFT;
	timings[2] = (timings[2]  & IW_NAND_SET_CYCLES_T2_MASK) <<
		IW_NAND_SET_CYCLES_T2_SHIFT;
	timings[3] = (timings[3]  & IW_NAND_SET_CYCLES_T3_MASK) <<
		IW_NAND_SET_CYCLES_T3_SHIFT;
	timings[4] = (timings[4] & IW_NAND_SET_CYCLES_T4_MASK) <<
		IW_NAND_SET_CYCLES_T4_SHIFT;
	timings[5]  = (timings[5]  & IW_NAND_SET_CYCLES_T5_MASK) <<
		IW_NAND_SET_CYCLES_T5_SHIFT;
	timings[6]  = (timings[6]  & IW_NAND_SET_CYCLES_T6_MASK) <<
		IW_NAND_SET_CYCLES_T6_SHIFT;
	timings[0] |= timings[1] | timings[2] | timings[3] |
		timings[4] | timings[5] | timings[6];

	writel(timings[0], xnfc->regs  + IW_NAND_SET_CYCLES_OFFS);
}
EXPORT_SYMBOL_GPL(iwave_smc_set_cycles);

/**
 * iwave_smc_ecc_is_busy - Read ecc busy flag
 * Return: the ecc_status bit from the ecc_status register. 1 = busy, 0 = idle
 */

bool iwave_smc_ecc_is_busy(struct iwave_nand_controller *xnfc)
{

	return ((readl(xnfc->regs + IW_NAND_ECC_STATUS_OFFS) &
				IW_NAND_ECC_STATUS_BUSY) == IW_NAND_ECC_STATUS_BUSY);
	return 0;
}
EXPORT_SYMBOL_GPL(iwave_smc_ecc_is_busy);

/**
 * iwave_smc_set_ecc_pg_size - Set SMC ECC page size
 * @pg_sz: ECC page size
 * Return: 0 on success or negative errno.
 */

int iwave_smc_set_ecc_pg_size(struct iwave_nand_controller *xnfc,
		unsigned int pg_sz)
{
	u32 reg, sz;

	switch (pg_sz) {
		case 0:
			sz = 0;
			break;
		case SZ_512:
			sz = 1;
			break;
		case SZ_1K:
			sz = 2;
			break;
		case SZ_2K:
			sz = 3;
			break;
		case SZ_4K:
			sz = 4;
			break;
		case SZ_8K:
			sz = 5;
			break;
		case SZ_16K:
			sz = 6;
			break;
		default:
			return -EINVAL;
	}

	reg = readl(xnfc->regs + IW_NAND_ECC_MEMCFG_OFFS);
	reg &= ~IW_NAND_ECC_MEMCFG_PGSIZE_MASK;
	reg |= sz;
	writel(reg, xnfc->regs + IW_NAND_ECC_MEMCFG_OFFS);
	return 0;
}
EXPORT_SYMBOL_GPL(iwave_smc_set_ecc_pg_size);

/**
 * iwave_smc_get_ecc_val - Read ecc_valueN registers
 * @ecc_reg: Index of the ecc_value reg (0..3)
 * Return: the content of the requested ecc_value register.
 *
 * There are four valid ecc_value registers. The argument is truncated to stay
 * within this valid boundary.
 */

u32 iwave_smc_get_ecc_val(struct iwave_nand_controller *xnfc, int ecc_reg)
{
	u32 addr, reg;

        addr = IW_NAND_ECC_VALUE0_OFFS + (ecc_reg * IW_NAND_ECC_REG_SIZE_OFFS);
        reg = readl(xnfc->regs + addr);
        return reg;
}
EXPORT_SYMBOL_GPL(iwave_smc_get_ecc_val);

/**
 * iwave_smc_get_nand_int_status_raw - Get NAND interrupt status bit
 * Return: the raw_int_status1 bit from the memc_status register
 */

int iwave_smc_get_nand_int_status_raw(struct iwave_nand_controller *xnfc,
		int poll_mode, unsigned long delay_us)
{
	u32 reg;
	u32 mask = 1 << IW_NAND_MEMC_STATUS_RAW_INT_1_SHIFT;
	int ret;

	if (poll_mode == POLL_MODE_BUSY) {
		reg = readl(xnfc->regs + IW_NAND_MEMC_STATUS_OFFS);
		reg >>= IW_NAND_MEMC_STATUS_RAW_INT_1_SHIFT;
		reg &= 1;
		ret = reg;
	} else {
		ret = readl_poll_timeout(xnfc->regs + IW_NAND_MEMC_STATUS_OFFS,
				reg, (reg & mask), delay_us, 1000000);
	}

	return ret;
}
EXPORT_SYMBOL_GPL(iwave_smc_get_nand_int_status_raw);

int iwave_read_error_reg(struct iwave_nand_controller *xnfc)
{
	u32 reg;

	reg = readl(xnfc->regs + IW_NAND_ADDR_TIMEOUT_ERROR);
	reg &= 1;
	return reg;
}
EXPORT_SYMBOL_GPL(iwave_read_error_reg);

/**
 * iwave_nand_init_nand_interface - Initialize the NAND interface
 * @adev: Pointer to the amba_device struct
 * @nand_node: Pointer to the iwave_nand device_node struct
 */

void iwave_nand_init_nand_interface(struct iwave_nand_controller *xnfc)
{
	unsigned long timeout;

	iwave_smc_set_buswidth(xnfc, IW_NAND_MEM_WIDTH_8);
	timeout = jiffies + IW_NAND_ECC_BUSY_TIMEOUT;
	/* Wait till the ECC operation is complete */
	do {
		if (iwave_smc_ecc_is_busy(xnfc))
			cpu_relax();
		else
			break;
	} while (!time_after_eq(jiffies, timeout));

	if (time_after_eq(jiffies, timeout))
		return;

	writel(IW_NAND_ECC_CMD1, xnfc->regs + IW_NAND_ECC_MEMCMD1_OFFS);
	writel(IW_NAND_ECC_CMD2, xnfc->regs + IW_NAND_ECC_MEMCMD2_OFFS);
}
EXPORT_SYMBOL_GPL(iwave_nand_init_nand_interface);
