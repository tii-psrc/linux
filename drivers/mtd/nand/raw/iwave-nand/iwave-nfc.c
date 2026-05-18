// SPDX-License-Identifier: GPL-2.0
/*
 *  iWave NAND flash controller driver
 *
 * Copyright (C) 2023 iWave Systems Technologies Pvt Ltd.
 *
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/moduleparam.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include "../internals.h"
#include <linux/pm_runtime.h>
#include <linux/mtd/nand-ecc-sw-hamming.h>
#include <linux/mtd/nand-ecc-sw-bch.h>
#include "iwave-lib.h"

#define IWAVE_NAND_DRIVER_NAME "iwave-nand"

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/kernel.h>

u32 set_timing = 0;

struct iwave_nfc_op {
	u32 cmnds[2];
	u32 addrs;
	unsigned int data_instr_idx;
	unsigned int rdy_timeout_ms;
	unsigned int rdy_delay_ns;
	const struct nand_op_instr *data_instr;
};

static void iWave_nand_select_target(struct nand_chip *chip, int num)
{
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);

	writel((num << 23), xnfc->regs + IW_NAND_DIRECT_CMD_OFFS);
}

static int iwave_ecc_ooblayout_ecc(struct mtd_info *mtd, int section,
		struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);

	if (section >= chip->ecc.steps)
		return -ERANGE;

	oobregion->length = chip->ecc.total;
	oobregion->offset = mtd->oobsize - oobregion->length;

	return 0;
}

static int iwave_ecc_ooblayout_free(struct mtd_info *mtd, int section,
		struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);

	if (section >= chip->ecc.steps)
		return -ERANGE;

	if (section)
		return -ERANGE;

	oobregion->offset = 2;
	oobregion->length = mtd->oobsize - chip->ecc.total - 2;

	return 0;
}

static const struct mtd_ooblayout_ops iwave_ecc_ooblayout_ops = {
	.ecc = iwave_ecc_ooblayout_ecc,
	.free = iwave_ecc_ooblayout_free,
};

static int iwave_ecc_ooblayout64_ecc(struct mtd_info *mtd, int section,
		struct mtd_oob_region *oobregion)
{
	struct nand_chip *chip = mtd_to_nand(mtd);

	if (section >= chip->ecc.steps)
		return -ERANGE;

	oobregion->offset = (section * chip->ecc.bytes) + 52;
	oobregion->length = chip->ecc.bytes;

	return 0;
}

static int iwave_ecc_ooblayout64_free(struct mtd_info *mtd, int section,
		struct mtd_oob_region *oobregion)
{
	if (section)
		return -ERANGE;

	oobregion->offset = 2;
	oobregion->length = 50;

	return 0;
}

static const struct mtd_ooblayout_ops iwave_ecc_ooblayout64_ops = {
	.ecc = iwave_ecc_ooblayout64_ecc,
	.free = iwave_ecc_ooblayout64_free,
};

/* Generic flash bbt decriptors */
static u8 bbt_pattern[] = { 'B', 'b', 't', '0' };
static u8 mirror_pattern[] = { '1', 't', 'b', 'B' };

static struct nand_bbt_descr bbt_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs = 4,
	.len = 4,
	.veroffs = 20,
	.maxblocks = 4,
	.pattern = bbt_pattern
};

static struct nand_bbt_descr bbt_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs = 4,
	.len = 4,
	.veroffs = 20,
	.maxblocks = 4,
	.pattern = mirror_pattern
};

static void iwave_nfc_force_byte_access(struct nand_chip *chip,
		bool force_8bit)
{
	int ret;

	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);

	if (xnfc->buswidth == 8)
		return;

	if (force_8bit)
		ret = iwave_smc_set_buswidth(xnfc, IW_NAND_MEM_WIDTH_8);
	else
		ret = iwave_smc_set_buswidth(xnfc, IW_NAND_MEM_WIDTH_16);

	if (ret)
		dev_err(xnfc->dev, "Error in Buswidth\n");
}

static inline int iwave_wait_for_dev_ready(struct iwave_nand_controller *xnfc,
		struct nand_chip *chip, int poll_mode, unsigned long delay_us)
{
	ktime_t start = ktime_get();
	u32 reg = 0;
	int ret;

	if (WARN_ON_ONCE(poll_mode != POLL_MODE_BUSY &&
				poll_mode != POLL_MODE_SCHEDULED))
		poll_mode = POLL_MODE_SCHEDULED;

	if (poll_mode == POLL_MODE_BUSY) {
		while (!(reg = iwave_smc_get_nand_int_status_raw(xnfc, poll_mode, 
						delay_us))) {
			if (ktime_ms_delta(ktime_get(), start) > 1000) {
				pr_debug("%s status(0x%08X)\n", __func__, reg);
				pr_err("%s timed out\n", __func__);
				return -ETIMEDOUT;
			}
			cpu_relax();
			udelay(delay_us);
		}
	} else if (poll_mode == POLL_MODE_SCHEDULED) {
		ret = iwave_smc_get_nand_int_status_raw(xnfc, poll_mode, delay_us);
		if (ret == -ETIMEDOUT) {
			pr_err("%s timed out\n", __func__);
			return ret;
		}
	}

	iwave_smc_clr_nand_int(xnfc);

	return 0;
}

/**
 * iwave_check_for_error - check for address timout error
 * @chip:       Pointer to the NAND chip info structure
 * Return:      returns 0 if the error bit is not set 
 *
 * We are polling the register for error bit until
 * the timeout occures. we are expecting a timeout 
 * for valid condition.*/
static inline int iwave_check_for_error(struct nand_chip *chip)
{
	unsigned long timeout = jiffies + IW_NAND_ERROR_TIMEOUT;
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);

	while (!iwave_read_error_reg(xnfc)) {
		if (time_after_eq(jiffies, timeout))
			return 0;
		cond_resched();
	}
	/* Clearing the address timeout error register */
	writel(IW_NAND_ADDR_TIMEOUT_ERROR_CLR, 
			xnfc->regs + IW_NAND_ADDR_TIMEOUT_ERROR);

	return -ENXIO;
}

/**
 * iwave_nand_read_data_op - read chip data into buffer
 * @chip:	Pointer to the NAND chip info structure
 * @in:		Pointer to the buffer to store read data
 * @len:	Number of bytes to read
 * @force_8bit:	Force 8-bit bus access
 * Return:	Always return zero
 */

static void iwave_nand_read_data_op(struct nand_chip *chip, u8 *in,
		unsigned int len, bool force_8bit)
{
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	int i;

	if (force_8bit)
		iwave_nfc_force_byte_access(chip, true);

	if ((IS_ALIGNED((uint32_t)in, sizeof(uint32_t)) &&
				IS_ALIGNED(len, sizeof(uint32_t))) || !force_8bit) {
		u32 *ptr = (u32 *)in;

		len /= 4;
		for (i = 0; i < len; i++)
			ptr[i] = readl(xnfc->nand_data + xnfc->dataphase_addrflags);
	} else {
		for (i = 0; i < len; i++)
			in[i] = readb(xnfc->nand_data + xnfc->dataphase_addrflags);
	}

	if (force_8bit)
		iwave_nfc_force_byte_access(chip, false);
}

/**
 * iwave_nand_write_data_op - write buffer to chip
 * @chip:	Pointer to the nand_chip structure
 * @buf:	Pointer to the buffer to store write data
 * @len:	Number of bytes to write
 * @force_8bit:	Force 8-bit bus access
 */

static void iwave_nand_write_data_op(struct nand_chip *chip, const u8 *buf,
		int len, bool force_8bit)
{
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	int i;

	if (force_8bit)
		iwave_nfc_force_byte_access(chip, true);

	if ((IS_ALIGNED((uint32_t)buf, sizeof(uint32_t)) &&
				IS_ALIGNED(len, sizeof(uint32_t))) || !force_8bit) {
		u32 *ptr = (u32 *)buf;

		len /= 4;
		for (i = 0; i < len; i++)
			writel(ptr[i], xnfc->nand_data + xnfc->dataphase_addrflags);
	} else {
		for (i = 0; i < len; i++)
			writeb(buf[i], xnfc->nand_data + xnfc->dataphase_addrflags);
	}

	if (force_8bit)
		iwave_nfc_force_byte_access(chip, false);
}

static inline int iwave_wait_for_ecc_done(struct nand_chip *chip)
{
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	unsigned long timeout = jiffies + IW_NAND_ECC_BUSY_TIMEOUT;

	while (iwave_smc_ecc_is_busy(xnfc)) {
		if (time_after_eq(jiffies, timeout)) {
			pr_err("%s timed out\n", __func__);
			return -ETIMEDOUT;
		}
		cond_resched();
	}

	return 0;
}

static void iwave_prepare_cmd(struct nand_chip *chip,
		int page, int column, int start_cmd, int end_cmd,
		bool read)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	unsigned long cmd_phase_data = 0;
	u32 end_cmd_valid = 0, cmdphase_addrflags;

	mdelay(1);

	end_cmd_valid = read ? 1 : 0;
	cmdphase_addrflags = ((xnfc->addr_cycles
				<< ADDR_CYCLES_SHIFT) |
			(end_cmd_valid << END_CMD_VALID_SHIFT) |
			(COMMAND_PHASE) |
			(end_cmd << END_CMD_SHIFT) |
			(start_cmd << START_CMD_SHIFT));

	/* Get the data phase address */
	xnfc->dataphase_addrflags = ((0x0 << CLEAR_CS_SHIFT) |
			(0 << END_CMD_VALID_SHIFT) |
			(DATA_PHASE) |
			(end_cmd << END_CMD_SHIFT) |
			(0x0 << ECC_LAST_SHIFT));

	if (chip->options & NAND_BUSWIDTH_16)
		column /= 2;

	cmd_phase_data = column;

	if(set_timing)
		cmd_phase_data |= 0x1;

	if (mtd->writesize > IW_NAND_ECC_SIZE) {
		cmd_phase_data |= page << 16;

		/* Another address cycle for devices > 128MiB */
		if (chip->options & NAND_ROW_ADDR_3) {
			writel_relaxed(cmd_phase_data,
					xnfc->nand_data + cmdphase_addrflags);
			cmd_phase_data = (page >> 16);
		}
	} else {
		cmd_phase_data |= page << 8;
	}

	writel_relaxed(cmd_phase_data, xnfc->nand_data + cmdphase_addrflags);
	set_timing = 0;
}

/**
 * iwave_nand_read_oob - [REPLACEABLE] the most common OOB data read function
 * @chip:	Pointer to the nand_chip structure
 * @chip:	Pointer to the nand_chip structure
 * @page:	Page number to read
 *
 * Return:	Always return zero
 */

static int iwave_nand_read_oob(struct nand_chip *chip,
		int page)
{
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	struct mtd_info *mtd = nand_to_mtd(chip);
	u8 *p;
	ktime_t s, e;

#if IS_ENABLED(CONFIG_MTD_NAND_HW_ECC_IWAVE)
	writel(0x0, xnfc->regs + IW_NAND_ECC_EL_DL_OFFS);
#else
	writel(0x1, xnfc->regs + IW_NAND_ECC_EL_DL_OFFS);
#endif
	if (mtd->writesize < IW_NAND_ECC_SIZE) {
		printk("iWave: NAND ECC size is greater than writesize\n");
		return 0;
	}

	writel((mtd->oobsize), xnfc->regs + IW_NAND_ADDR_SIZE_DATA);
	iwave_prepare_cmd(chip, page, mtd->writesize, NAND_CMD_READ0, NAND_CMD_READSTART, 1);

	s = ktime_get();
	if (iwave_wait_for_dev_ready(xnfc, chip, POLL_MODE_BUSY, 10))
		return -ETIMEDOUT;
	e = ktime_get();
	//printk("%s: took %lld us\n", __func__, ktime_us_delta(e, s));

	p = chip->oob_poi;
	iwave_nand_read_data_op(chip, p, (mtd->oobsize), false);

	return 0;
}

/**
 * iwave_nand_write_oob - [REPLACEABLE] the most common OOB data write function
 * @chip:	Pointer to the nand_chip structure
 * @chip:	Pointer to the NAND chip info structure
 * @page:	Page number to write
 *
 * Return:	Zero on success and EIO on failure
 */
static int iwave_nand_write_oob(struct nand_chip *chip, int page)
{
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	struct mtd_info *mtd = nand_to_mtd(chip);
	const u8 *buf = chip->oob_poi;
	ktime_t s, e;

#if IS_ENABLED(CONFIG_MTD_NAND_HW_ECC_IWAVE)
	writel(0x0, xnfc->regs + IW_NAND_ECC_EL_DL_OFFS);
	writel((mtd->oobsize - (chip->ecc.bytes * chip->ecc.steps)), xnfc->regs + IW_NAND_ADDR_SIZE_DATA);
#else
	writel(0x1, xnfc->regs + IW_NAND_ECC_EL_DL_OFFS);
	writel(mtd->oobsize, xnfc->regs + IW_NAND_ADDR_SIZE_DATA);
#endif
	iwave_prepare_cmd(chip, page, mtd->writesize, NAND_CMD_SEQIN, NAND_CMD_PAGEPROG, 0);

#if IS_ENABLED(CONFIG_MTD_NAND_HW_ECC_IWAVE)
	iwave_nand_write_data_op(chip, buf, (mtd->oobsize - (chip->ecc.bytes * chip->ecc.steps)), false);
#else
	iwave_nand_write_data_op(chip, buf, mtd->oobsize, false);
#endif
	s = ktime_get();
	if (iwave_wait_for_dev_ready(xnfc, chip, POLL_MODE_SCHEDULED, 100))
		return -ETIMEDOUT;
	e = ktime_get();
	//printk("%s: took %lld us\n", __func__, ktime_us_delta(e, s));

	return 0;
}

/**
 * iwave_nand_read_page_raw - [Intern] read page data without ecc
 * @chip:		Pointer to the nand_chip structure
 * @buf:		Pointer to the data buffer
 * @oob_required:	Caller requires OOB data read to chip->oob_poi
 * @page:		Page number to read
 *
 * Return:	Always return zero
 */
static int iwave_nand_read_page_raw(struct nand_chip *chip, u8 *buf,
				int oob_required, int page)
{
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	struct mtd_info *mtd = nand_to_mtd(chip);
	u8 *p;
	ktime_t s, e;

	writel(0x1, xnfc->regs + IW_NAND_ECC_EL_DL_OFFS);
	writel(mtd->writesize + (mtd->oobsize), xnfc->regs + IW_NAND_ADDR_SIZE_DATA);
	iwave_prepare_cmd(chip, page, 0, NAND_CMD_READ0, NAND_CMD_READSTART, 1);
	s = ktime_get();
	if (iwave_wait_for_dev_ready(xnfc, chip, READ_ONCE(xnfc->poll_mode), 10))
		return -ETIMEDOUT;
	e = ktime_get();
	//printk("%s: took %lld us\n", __func__, ktime_us_delta(e, s));

	if (!buf)
		return 0;

	iwave_nand_read_data_op(chip, buf, mtd->writesize, false);
	p = chip->oob_poi;
	iwave_nand_read_data_op(chip, p, mtd->oobsize, false);

	return 0;
}

/**
 * iwave_nand_write_page_raw - [Intern] raw page write function
 * @chip:		Pointer to the nand_chip structure
 * @buf:		Pointer to the data buffer
 * @oob_required:	Caller requires OOB data read to chip->oob_poi
 * @page:		Page number to write
 *
 * Return:	Always return zero
 */
static int iwave_nand_write_page_raw(struct nand_chip *chip, const u8 *buf,
				int oob_required, int page)
{
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	struct mtd_info *mtd = nand_to_mtd(chip);
	u8 *p;
	ktime_t s, e;

	writel(0x1, xnfc->regs + IW_NAND_ECC_EL_DL_OFFS);
	writel(mtd->writesize + (mtd->oobsize), xnfc->regs + IW_NAND_ADDR_SIZE_DATA);
	iwave_prepare_cmd(chip, page, 0, NAND_CMD_SEQIN, NAND_CMD_PAGEPROG, 0);

	iwave_nand_write_data_op(chip, buf, mtd->writesize, false);
	p = chip->oob_poi;
	iwave_nand_write_data_op(chip, p, mtd->oobsize, false);

	s = ktime_get();
	if (iwave_wait_for_dev_ready(xnfc, chip, POLL_MODE_SCHEDULED, 100))
		return -ETIMEDOUT;
	e = ktime_get();
	//printk("%s: took %lld us\n", __func__, ktime_us_delta(e, s));

	return 0;
}

#if IS_ENABLED(CONFIG_MTD_NAND_HW_ECC_IWAVE)
/**
 * nand_write_page_hwecc - Hardware ECC based page write function
 * @chip:		Pointer to the nand_chip structure
 * @buf:		Pointer to the data buffer
 * @oob_required:	Caller requires OOB data read to chip->oob_poi
 * @page:		Page number to write
 *
 * This functions writes data and hardware generated ECC values in to the page.
 *
 * Return:	Always return zero
 */
static int iwave_nand_write_page_hwecc(struct nand_chip *chip, const u8 *buf,
		int oob_required, int page)
{
	int eccsize = chip->ecc.size;
	int eccsteps = chip->ecc.steps;
	u8 *oob_ptr;
	const u8 *p = buf;
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	struct mtd_info *mtd = nand_to_mtd(chip);

	writel(0x0, xnfc->regs + IW_NAND_ECC_EL_DL_OFFS);
	writel((eccsteps * eccsize) + (mtd->oobsize - (3 * chip->ecc.steps)),
			xnfc->regs + IW_NAND_ADDR_SIZE_DATA);
	iwave_prepare_cmd(chip, page, 0, NAND_CMD_SEQIN, NAND_CMD_PAGEPROG, 0);

	for ( ; (eccsteps); eccsteps--) {
		iwave_nand_write_data_op(chip, p, eccsize, false);
		p += eccsize;
	}

       /* Write the spare area without ECC bytes */
       oob_ptr = chip->oob_poi;
       iwave_nand_write_data_op(chip, oob_ptr, (mtd->oobsize - (3 * chip->ecc.steps)), false);

       if (iwave_wait_for_dev_ready(xnfc, chip))
               return -ETIMEDOUT;


	return 0;
}

/**
 * iwave_nand_read_page_hwecc - Hardware ECC based page read function
 * @chip:		Pointer to the nand_chip structure
 * @buf:		Pointer to the buffer to store read data
 * @oob_required:	Caller requires OOB data read to chip->oob_poi
 * @page:		Page number to read
 *
 * This functions reads data and checks the data integrity by comparing
 * hardware generated ECC values and read ECC values from spare area.
 * There is a limitation in SMC controller, that we must set ECC LAST on
 * last data phase access, to tell ECC block not to expect any data further.
 * Ex:  When number of ECC STEPS are 4, then till 3 we will write to flash
 * using SMC with HW ECC enabled. And for the last ECC STEP, we will subtract
 * 4bytes from page size, and will initiate a transfer. And the remaining 4 as
 * one more transfer with ECC_LAST bit set in NAND data phase register to
 * notify ECC block not to expect any more data. The last block should be align
 * with end of 512 byte block. Because of this limitation, we are not using
 * core routines.
 *
 * Return:	0 always and updates ECC operation status in to MTD structure
 */

static int iwave_nand_read_page_hwecc(struct nand_chip *chip,
		u8 *buf, int oob_required, int page)
{
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	struct mtd_info *mtd = nand_to_mtd(chip);
	int i, stat, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	unsigned int max_bitflips = 0;
	unsigned int failed = 0, bitflips = 0;
	u8 *p = buf;
	u8 *ecc = chip->ecc.code_buf;
	u8 *oob_ptr;

	writel(0x0, xnfc->regs + IW_NAND_ECC_EL_DL_OFFS);
	writel((eccsteps * eccsize) + (mtd->oobsize), xnfc->regs + IW_NAND_ADDR_SIZE_DATA);
	iwave_prepare_cmd(chip, page, 0, NAND_CMD_READ0, NAND_CMD_READSTART, 1);

	if (iwave_wait_for_dev_ready(xnfc, chip))
		return -ETIMEDOUT;

	for ( ; (eccsteps); eccsteps--) {
		iwave_nand_read_data_op(chip, p, eccsize, false);
		p += eccsize;
	}

	stat = readl(xnfc->regs + IW_NAND_ECC_ERR);

	failed = (stat >> 16);
	bitflips = (stat & 0xFFFF);
	stat =0;

	/* Read the stored ECC value */
	oob_ptr = chip->oob_poi;
	iwave_nand_read_data_op(chip, oob_ptr, (mtd->oobsize), false);

	if (failed != 0) {
		mtd->ecc_stats.failed++;
		max_bitflips = 2;
	} else if (bitflips != 0){
		max_bitflips = 1;
		while (bitflips) {
			mtd->ecc_stats.corrected++;
			bitflips &= (bitflips - 1);
		}
	}

	return max_bitflips;
}
#endif

/**
 * iwave_nand_write_page_swecc - BCH software ECC based page write function
 * @chip: nand chip info structure
 * @buf: data buffer
 * @oob_required: must write chip->oob_poi to OOB
 * @page: page number to write
 */
static int iwave_nand_write_page_swecc(struct nand_chip *chip, const uint8_t *buf,
				int oob_required, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	int i, eccsize = chip->ecc.size, ret;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *ecc_calc = chip->ecc.calc_buf;
	const uint8_t *p = buf;

	/* Software ECC calculation */
	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize)
		chip->ecc.calculate(chip, p, &ecc_calc[i]);

	ret = mtd_ooblayout_set_eccbytes(mtd, ecc_calc, chip->oob_poi, 0,
			chip->ecc.total);
	if (ret)
		return ret;

	return chip->ecc.write_page_raw(chip, buf, 1, page);
}

/**
 * iwave_nand_read_page_swecc - BCH software ECC based page read function
 * @chip: nand chip info structure
 * @buf: buffer to store read data
 * @oob_required: caller requires OOB data read to chip->oob_poi
 * @page: page number to read
 */
static int iwave_nand_read_page_swecc(struct nand_chip *chip, uint8_t *buf,
		int oob_required, int page)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	int i, eccsize = chip->ecc.size, ret;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *p = buf;
	uint8_t *ecc_calc = chip->ecc.calc_buf;
	uint8_t *ecc_code = chip->ecc.code_buf;
	unsigned int max_bitflips = 0;

        chip->ecc.read_page_raw(chip, buf, 1, page);

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize)
		chip->ecc.calculate(chip, p, &ecc_calc[i]);

	ret = mtd_ooblayout_get_eccbytes(mtd, ecc_code, chip->oob_poi, 0, chip->ecc.total);
	if (ret)
		return ret;

	eccsteps = chip->ecc.steps;
	p = buf;

	for (i = 0 ; eccsteps; eccsteps--, i += eccbytes, p += eccsize) {
		int stat;

		stat = chip->ecc.correct(chip, p, &ecc_code[i], &ecc_calc[i]);
		if (stat < 0) {
			mtd->ecc_stats.failed++;
		} else {
			mtd->ecc_stats.corrected += stat;
			max_bitflips = max_t(unsigned int, max_bitflips, stat);
		}
        }
        return max_bitflips;
}
static int iwave_nand_exec_op_cmd(struct nand_chip *chip, const struct nand_subop *subop)
{
	struct iwave_nfc_op nfc_op = {};
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	unsigned long end_cmd_valid = 0;
	unsigned int op_id, len;
	bool reading;
	u32 cmdphase_addrflags;
	const struct nand_op_instr *instr = NULL;
	int i;
	u32 col = 0, row = 0;
	u32 naddrs = 0;

	memset(&nfc_op, 0, sizeof(struct iwave_nfc_op));
	for (op_id = 0; op_id < subop->ninstrs; op_id++) {
		instr = &subop->instrs[op_id];

		switch (instr->type) {
			case NAND_OP_CMD_INSTR:
				if (op_id) {
					nfc_op.cmnds[1] = instr->ctx.cmd.opcode;

					/*
					 * end_cmd_valid is set when there is a
					 * command cycle followed by Address cycle
					 */
					if (naddrs)
						end_cmd_valid = 1;
				} else {
					nfc_op.cmnds[0] = instr->ctx.cmd.opcode;
					end_cmd_valid = 0;
				}

				break;

			case NAND_OP_ADDR_INSTR:
				i = nand_subop_get_addr_start_off(subop, op_id);
				naddrs = nand_subop_get_num_addr_cyc(subop,
						op_id);
				for (i = 0; i < min_t(unsigned int, 4, naddrs); i++)
					col |= instr->ctx.addr.addrs[i] << (8 * i);

				if (naddrs >= 5)
					row = instr->ctx.addr.addrs[4];

				if (naddrs >= 6)
					row |= (instr->ctx.addr.addrs[5] << 8);

				break;

			case NAND_OP_DATA_IN_INSTR:
			case NAND_OP_DATA_OUT_INSTR:
				nfc_op.data_instr = instr;
				nfc_op.data_instr_idx = op_id;
				break;

			case NAND_OP_WAITRDY_INSTR:
				nfc_op.rdy_timeout_ms = instr->ctx.waitrdy.timeout_ms;
				nfc_op.rdy_delay_ns = instr->delay_ns;
#if 0
				if (instr->ctx.cmd.opcode)
					printk("%s, instr->ctx.cmd.opcode(%d)\n",
							__func__, instr->ctx.cmd.opcode);
				printk("%s, nfc_op.rdy_timeout_ms(%d)\n",
						__func__, nfc_op.rdy_timeout_ms);
				printk("%s, nfc_op.rdy_delay_ns (%d)\n",
						__func__, nfc_op.rdy_delay_ns);
#endif
				break;
		}
	}

	instr = nfc_op.data_instr;
	op_id = nfc_op.data_instr_idx;

	/* Clear interrupts */
	iwave_smc_clr_nand_int(xnfc);

	cmdphase_addrflags = ((naddrs << ADDR_CYCLES_SHIFT) |
			(end_cmd_valid << END_CMD_VALID_SHIFT) |
			(COMMAND_PHASE) |
			(nfc_op.cmnds[1] << END_CMD_SHIFT) |
			(nfc_op.cmnds[0] << START_CMD_SHIFT));

	xnfc->dataphase_addrflags = ((0x0 << CLEAR_CS_SHIFT) |
			(0 << END_CMD_VALID_SHIFT) |
			(DATA_PHASE) |
			(nfc_op.cmnds[0] << END_CMD_SHIFT) |
			(0x0 << ECC_LAST_SHIFT));


	if (naddrs > 3) {
		writel_relaxed(col, xnfc->nand_data + cmdphase_addrflags);
		writel_relaxed(row, xnfc->nand_data + cmdphase_addrflags);
	} else {
		writel_relaxed(col, xnfc->nand_data + cmdphase_addrflags);
	}

	if (!nfc_op.data_instr) {
		if (nfc_op.rdy_timeout_ms) {
			mdelay(12);
			if (iwave_wait_for_dev_ready(xnfc, chip, POLL_MODE_BUSY, 10))
				return -ETIMEDOUT;
		}
		return 0;
	}

	reading = (nfc_op.data_instr->type == NAND_OP_DATA_IN_INSTR);
	len = nand_subop_get_data_len(subop, op_id);

	if (!reading) {
		iwave_nand_write_data_op(chip, instr->ctx.data.buf.out,
				len, instr->ctx.data.force_8bit);
		if (nfc_op.rdy_timeout_ms) {
			mdelay(12);
			if (iwave_wait_for_dev_ready(xnfc, chip, POLL_MODE_BUSY, 10))
				return -ETIMEDOUT;
		}
		ndelay(nfc_op.rdy_delay_ns);
	} else {
		ndelay(nfc_op.rdy_delay_ns);

		if (nfc_op.rdy_timeout_ms) {
			mdelay(12);
			if (iwave_wait_for_dev_ready(xnfc, chip, POLL_MODE_BUSY, 10))
				return -ETIMEDOUT;
		}

		/* Read ID is a part of device detection. 
		 * We are checking the error status for	
		 * Read-ID command. this will help us to 
		 * check whether the NAND is present or not.*/
		if(nfc_op.cmnds[0] == NAND_CMD_READID)
		{
			if(iwave_check_for_error(chip))
				return -ENXIO;
		}
		iwave_nand_read_data_op(chip, instr->ctx.data.buf.in, len,
				instr->ctx.data.force_8bit);
	}

	return 0;
}

static const struct nand_op_parser iwave_nfc_op_parser = NAND_OP_PARSER(
		NAND_OP_PARSER_PATTERN(iwave_nand_exec_op_cmd,
			NAND_OP_PARSER_PAT_CMD_ELEM(true),
			NAND_OP_PARSER_PAT_ADDR_ELEM(true, 7),
			NAND_OP_PARSER_PAT_CMD_ELEM(true),
			NAND_OP_PARSER_PAT_WAITRDY_ELEM(true),
			NAND_OP_PARSER_PAT_DATA_IN_ELEM(true, IW_MAX_CHUNK_SIZE)),
		NAND_OP_PARSER_PATTERN(iwave_nand_exec_op_cmd,
			NAND_OP_PARSER_PAT_CMD_ELEM(true),
			NAND_OP_PARSER_PAT_ADDR_ELEM(true, 7),
			NAND_OP_PARSER_PAT_DATA_OUT_ELEM(true, IW_MAX_CHUNK_SIZE),
			NAND_OP_PARSER_PAT_CMD_ELEM(true),
			NAND_OP_PARSER_PAT_WAITRDY_ELEM(true)),
		NAND_OP_PARSER_PATTERN(iwave_nand_exec_op_cmd,
			NAND_OP_PARSER_PAT_DATA_OUT_ELEM(false, IW_MAX_CHUNK_SIZE),
			NAND_OP_PARSER_PAT_WAITRDY_ELEM(true)),
		);

static int iwave_nfc_exec_op(struct nand_chip *chip,
		const struct nand_operation *op,
		bool check_only)
{
	nand_select_target(chip, op->cs);
	return nand_op_parser_exec_op(chip, &iwave_nfc_op_parser,
			op, check_only);
}

/**
 * iwave_nand_ecc_init - Initialize the ecc information as per the ecc mode
 * @mtd:	Pointer to the mtd_info structure
 * @ecc:	Pointer to ECC control structure
 * @ecc_mode:	ondie ecc status
 *
 * This function initializes the ecc block and functional pointers as per the
 * ecc mode
 *
 * Return:	0 on success or negative errno.
 */

static int iwave_nand_ecc_init(struct mtd_info *mtd, struct nand_ecc_ctrl *ecc,
		int ecc_mode)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	int ret = 0;

	ecc->read_oob = iwave_nand_read_oob;
	ecc->write_oob = iwave_nand_write_oob;
	ecc->write_page_raw = iwave_nand_write_page_raw;
	ecc->read_page_raw = iwave_nand_read_page_raw;

	if (ecc_mode == NAND_ECC_ENGINE_TYPE_ON_DIE) {
		ecc->write_page = iwave_nand_write_page_raw;
		ecc->read_page = iwave_nand_read_page_raw;

		/*
		 * On-Die ECC spare bytes offset 8 is used for ECC codes
		 * Use the BBT pattern descriptors
		 */
		chip->bbt_td = &bbt_main_descr;
		chip->bbt_md = &bbt_mirror_descr;
		ret = iwave_smc_set_ecc_mode(xnfc, IW_NAND_ECCMODE_BYPASS);
		if (ret)
			return ret;

	} else {
#if IS_ENABLED(CONFIG_MTD_NAND_HW_ECC_IWAVE)
		ecc->engine_type = NAND_ECC_ENGINE_TYPE_ON_HOST;

		/* Hardware ECC generates 3 bytes ECC code for each 512 bytes */
		ecc->bytes = 3;
		ecc->strength = 1;
		ecc->size = IW_NAND_ECC_SIZE;
		ecc->read_page = iwave_nand_read_page_hwecc;
		ecc->write_page = iwave_nand_write_page_hwecc;
		mtd_set_ooblayout(mtd, &iwave_ecc_ooblayout_ops);
#endif

#if IS_ENABLED(CONFIG_MTD_NAND_SW_ECC_IWAVE)
		ecc->engine_type = NAND_ECC_ENGINE_TYPE_SOFT;
		ecc->algo = NAND_ECC_ALGO_BCH;
		ecc->bytes = 13;
		ecc->strength = 8;
		ecc->read_page = iwave_nand_read_page_swecc;
		ecc->write_page = iwave_nand_write_page_swecc;
		ecc->size = IW_NAND_ECC_SIZE;
		mtd_set_ooblayout(mtd, nand_get_large_page_ooblayout());
#endif

#if IS_ENABLED(CONFIG_MTD_NAND_RAW_IWAVE)
		ecc->engine_type = NAND_ECC_ENGINE_TYPE_NONE;
		ecc->write_page = iwave_nand_write_page_raw;
		ecc->read_page = iwave_nand_read_page_raw;
		ecc->size = mtd->writesize;
		ecc->bytes = 0;
		ecc->strength = 0;
		mtd_set_ooblayout(mtd, &iwave_ecc_ooblayout_ops);
#endif

		writel(IW_NAND_CFG_CLR_DEFAULT_MASK,
				xnfc->regs + IW_NAND_ADDR_SIZE_PAGE_OOB);
	}

	return ret;
}

static int iWave_init_timing_mode(struct nand_chip *chip, int targets)
{
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	int mode;
	u8 feature[ONFI_SUBFEATURE_PARAM_LEN];
	u32 i; 

	mode = fls(chip->parameters.onfi->nvddr_timing_modes) & 0xff;
	if (!mode) {
		mode = fls(chip->parameters.onfi->sdr_timing_modes) - 1;
	}

	printk("Configuring NAND mode %d\n", mode);
	writel(mode, xnfc->regs + IW_NAND_TIMING_SYNC_ASYNC);
	memset(feature, 0, sizeof(feature));
	feature[0] = mode;
	for(i =0; i < targets; i++){
		nand_select_target(chip, i); 
		set_timing = 1;
		iwave_prepare_cmd(chip, 0, 0, NAND_CMD_SET_FEATURES, 0, 1);
		iwave_nand_write_data_op(chip, feature, sizeof(feature), false);
		nand_deselect_target(chip);
	}

	return 0;
}

static int iwave_nfc_setup_data_interface(struct nand_chip *chip, int csline,
		const struct nand_interface_config *conf)
{
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	u32 timings[7];

	timings[0] = 4 ; /*t_Rc_min*/
	timings[1] = 4 ; /*t_Wc_min*/
	timings[2] = 1 ; /*t_Rea*/
	timings[3] = 2 ; /*t_Wp */
	timings[4] = 2 ; /*t_Clr*/
	timings[5] = 2 ; /*t_Ar*/
	timings[6] = 4 ; /*t_Rr*/

	iwave_smc_set_cycles(xnfc, timings);

	return 0;
}

static int iwave_nand_attach_chip(struct nand_chip *chip)
{
	struct mtd_info *mtd = nand_to_mtd(chip);
	struct iwave_nand_controller *xnfc = to_iwave_nand(chip->controller);
	int ret;

	if (chip->options & NAND_BUSWIDTH_16) {
		ret = iwave_smc_set_buswidth(xnfc, IW_NAND_MEM_WIDTH_16);
		if (ret) {
			dev_err(xnfc->dev, "Set BusWidth failed\n");
			return ret;
		}
	}

	if (mtd->writesize <= SZ_512)
		xnfc->addr_cycles = 1;
	else
		xnfc->addr_cycles = 2;

	if (chip->options & NAND_ROW_ADDR_3)
		xnfc->addr_cycles += 3;
	else
		xnfc->addr_cycles += 2;

	chip->ecc.code_buf = kmalloc(mtd->oobsize, GFP_KERNEL);

	ret = iwave_nand_ecc_init(mtd, &chip->ecc, chip->ecc.engine_type);
	if (ret) {
		dev_err(xnfc->dev, "ECC init failed\n");
		return ret;
	}

	return 0;
}

static const struct nand_controller_ops iwave_nand_controller_ops = {
	.attach_chip = iwave_nand_attach_chip,
	.exec_op = iwave_nfc_exec_op,
	.setup_interface = iwave_nfc_setup_data_interface,
};

static int iwave_nand_chip_init(struct iwave_nand_controller *xnfc,
		struct iwave_nand_chip *inand_chip,
		struct device_node *np)
{
	struct nand_chip *chip = &inand_chip->chip;
	struct mtd_info *mtd = nand_to_mtd(chip);
	int ret;

	ret = of_property_read_u32(np, "cs_num", &inand_chip->csnum);
	if (ret) {
		dev_err(xnfc->dev, "can't get chip-select\n");
		return -ENXIO;
	}
	mtd->name = devm_kasprintf(xnfc->dev, GFP_KERNEL, "iWave_nand.%08llx",
			xnfc->flash_reg->start);
	mtd->dev.parent = xnfc->dev;
	chip->controller = &xnfc->controller;
	chip->options = NAND_BUSWIDTH_AUTO | NAND_NO_SUBPAGE_WRITE;
#if 1
	chip->bbt_options = NAND_BBT_CREATE;
#else
	chip->bbt_options = NAND_BBT_USE_FLASH;
#endif
	chip->legacy.select_chip = iWave_nand_select_target;
	nand_set_flash_node(chip, np);
	ret = nand_scan(chip, inand_chip->csnum);
	if (ret) {
		dev_err(xnfc->dev, "nand_scan_tail for NAND failed\n");
		return ret;
	}
	ret = iWave_init_timing_mode(chip, inand_chip->csnum);
	if (ret) {
		printk("timing mode init failed\n");
		return ret;
	}

	return mtd_device_register(mtd, NULL, 0);
}

static int poll_mode_proc_show(struct seq_file *m, void *v)
{
	struct iwave_nand_controller *xnfc = m->private;
	int val;

	spin_lock(&xnfc->lock);
	val = xnfc->poll_mode;
	spin_unlock(&xnfc->lock);

	seq_printf(m, "%d (%s)\n",
			val,
			val == POLL_MODE_BUSY ?
			"busy" : "scheduled");

	return 0;
}

static int poll_mode_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, poll_mode_proc_show, pde_data(inode));
}

static ssize_t poll_mode_proc_write(struct file *file,
		const char __user *buf,
		size_t count,
		loff_t *ppos)
{
	struct iwave_nand_controller *xnfc = pde_data(file_inode(file));
	char tmp[16];
	int val, ret;

	if (count >= sizeof(tmp))
		return -EINVAL;

	if (copy_from_user(tmp, buf, count))
		return -EFAULT;

	tmp[count] = '\0';

	ret = kstrtoint(tmp, 10, &val);
	if (ret)
		return ret;

	if (val < POLL_MODE_SCHEDULED ||
			val > POLL_MODE_BUSY)
		return -EINVAL;

	spin_lock(&xnfc->lock);
	xnfc->poll_mode = val;
	spin_unlock(&xnfc->lock);

	return count;
}

static const struct proc_ops poll_mode_proc_ops = {
	.proc_open    = poll_mode_proc_open,
	.proc_read    = seq_read,
	.proc_lseek   = seq_lseek,
	.proc_release = single_release,
	.proc_write   = poll_mode_proc_write,
};

static int iwave_nand_proc_init(struct iwave_nand_controller *xnfc,
		struct mtd_info *mtd)
{
	xnfc->proc_dir = proc_mkdir(mtd->name, NULL);
	if (!xnfc->proc_dir)
		return -ENOMEM;

	proc_create_data("poll_mode", 0644, xnfc->proc_dir, &poll_mode_proc_ops,
			xnfc);

	return 0;
}

static void iwave_nand_proc_remove(struct mtd_info *mtd)
{
	remove_proc_subtree(mtd->name, NULL);
}

static int cmdline_poll_mode = POLL_MODE_UNKNOWN;
static int __init poll_mode_setup(char *str)
{
	if (!str)
		return 0;

	kstrtoint(str, 0, &cmdline_poll_mode);

	if (WARN_ON_ONCE(cmdline_poll_mode != POLL_MODE_BUSY &&
				cmdline_poll_mode != POLL_MODE_SCHEDULED))
		cmdline_poll_mode = POLL_MODE_BUSY;

	printk("[%s] cmdline_poll_mode=%d\n", __func__, cmdline_poll_mode);

	return 1;
}
__setup("poll_mode=", poll_mode_setup);

/**
 * iwave_nand_probe - Probe method for the NAND driver
 * @pdev:	Pointer to the platform_device structure
 *
 * This function initializes the driver data structures and the hardware.
 * The NAND driver has dependency with the iwave memory controller
 * driver for initializing the NAND timing parameters, bus width, ECC modes,
 * control and status information.
 *
 * Return:	0 on success or error value on failure
 */

static int iwave_nand_probe(struct platform_device *pdev)
{
	struct iwave_nand_controller *xnfc;
	struct nand_chip *chip = NULL;
	struct device_node *np = pdev->dev.of_node;
	u32 val = 0, err;

	printk("iWave NAND driver started loading\n");

	xnfc = devm_kzalloc(&pdev->dev, sizeof(*xnfc), GFP_KERNEL);
	if (!xnfc)
		return -ENOMEM;

	nand_controller_init(&xnfc->controller);

	xnfc->dev = &pdev->dev;
	xnfc->controller.ops = &iwave_nand_controller_ops;
	/* Map physical address of NAND flash */
	xnfc->flash_reg = platform_get_resource_byname(pdev, IORESOURCE_MEM,"nand_reg");
	xnfc->regs = devm_ioremap_resource(xnfc->dev, xnfc->flash_reg);

	if (IS_ERR(xnfc->regs))
		return PTR_ERR(xnfc->regs);

	xnfc->flash_data = platform_get_resource_byname(pdev, IORESOURCE_MEM,"nand_data");
	xnfc->nand_data = devm_ioremap_resource(xnfc->dev, xnfc->flash_data);

	if (IS_ERR(xnfc->nand_data))
		return PTR_ERR(xnfc->nand_data);


	/* clear interrupts */
	writel(IW_NAND_CFG_CLR_DEFAULT_MASK, xnfc->regs + IW_NAND_CFG_CLR_OFFS);

	iwave_nand_init_nand_interface(xnfc);

	val = 8;
	xnfc->buswidth = val;

	/* Set the device option and flash width */
	platform_set_drvdata(pdev, xnfc);

#if IS_ENABLED(CONFIG_MTD_NAND_HW_ECC_IWAVE)
	writel(0x0, xnfc->regs + IW_NAND_ECC_EL_DL_OFFS);
#else
	writel(0x1, xnfc->regs + IW_NAND_ECC_EL_DL_OFFS);
#endif

	xnfc->inand_chip = devm_kzalloc(&pdev->dev, sizeof(*xnfc->inand_chip),
			GFP_KERNEL);
	if (!xnfc->inand_chip) {
		return -ENOMEM;
	}
	/* Set the device option and flash width */
	err = iwave_nand_chip_init(xnfc, xnfc->inand_chip, np);
	if (err) 
		devm_kfree(&pdev->dev, xnfc->inand_chip);

	spin_lock_init(&xnfc->lock);
	xnfc->poll_mode = (cmdline_poll_mode == POLL_MODE_UNKNOWN) ?
		POLL_MODE_BUSY : POLL_MODE_SCHEDULED;
	chip = &xnfc->inand_chip->chip;
	err = iwave_nand_proc_init(xnfc, nand_to_mtd(chip));
	if (err)
		return err;

	return 0;
}

/**
 * iwave_nand_remove - Remove method for the NAND driver
 * @pdev:	Pointer to the platform_device structure
 *
 * This function is called if the driver module is being unloaded. It frees all
 * resources allocated to the device.
 *
 * Return:	0 on success or error value on failure
 */

static int iwave_nand_remove(struct platform_device *pdev)
{
	struct iwave_nand_controller *xnfc = platform_get_drvdata(pdev);;
	struct nand_chip *chip = &xnfc->inand_chip->chip;
	struct mtd_info *mtd = nand_to_mtd(chip);

	iwave_nand_proc_remove(mtd);
	/* Release resources, unregister device */
	nand_cleanup(chip);

	return 0;
}

/* Match table for device tree binding */
static const struct of_device_id iwave_nand_of_match[] = {
	{ .compatible = "iw,iwave-nand-dt" },
	{},
};
MODULE_DEVICE_TABLE(of, iwave_nand_of_match);

/*
 * iwave_nand_driver - This structure defines the NAND subsystem platform driver
 */
static struct platform_driver iwave_nand_driver = {
	.probe		= iwave_nand_probe,
	.remove		= iwave_nand_remove,
	.driver		= {
		.name	= IWAVE_NAND_DRIVER_NAME,
		.of_match_table = iwave_nand_of_match,
	},
};

module_platform_driver(iwave_nand_driver);

MODULE_AUTHOR("iWave Systems Technologies");
MODULE_ALIAS("platform:" IWAVE_NAND_DRIVER_NAME);
MODULE_DESCRIPTION("iWave NAND Flash Controller Driver");
MODULE_LICENSE("GPL");
