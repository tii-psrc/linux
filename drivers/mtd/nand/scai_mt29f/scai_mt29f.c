// SPDX-License-Identifier: GPL-2.0
/*
 * mchp_scai_nand.c
 *
 * MTD NAND driver for Micron MT29F flash attached to a custom SCAI QSPI
 * controller.
 *
 * (Dual-compilation support for U-Boot DM and Linux Kernel Platform Driver)
 */

/* ====================================================================== */
/* U-BOOT / LINUX KERNEL COMPATIBILITY HEADER                         */
/* ====================================================================== */
#ifdef __UBOOT__
#include <common.h>
#include <dm.h>
#include <fdtdec.h>
#include <log.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/mtd/scai_mt29f_regs.h>
#include <dm/device.h>
#include <dm/device_compat.h>
#include <dm/uclass.h>
#else
/* Linux Kernel Includes */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/mutex.h>
#include "scai_mt29f_regs.h" /* Assumes header is in include/linux/mtd */
#endif

#define GPIO_REG_WDATA_OFFSET   0x00
#define GPIO_REG_RDATA_OFFSET   0x04
#define GPIO1_ENA_SS1_MASK      BIT(4)
#define GPIO2_ENA_SS2_MASK      BIT(0)

#define MT29F_JEDEC_MANUFACTURER_ID 0x2C
#define MT29F_JEDEC_DEVICE_ID_0     0x46

#define MT29F_PAGE_SIZE			4096
#define MT29F_OOB_SIZE			256
#define MT29F_PAGES_PER_BLOCK		64
#define MT29F_BLOCKS_PER_DIE		2048
#define MT29F_LUNS_PER_DIE		1
#define MT29F_BITS_PER_CELL		1

/**
 * struct scai_nand_priv - Private driver data structure
 */
struct scai_nand_priv {
	struct nand_device nand;
	struct mtd_info mtd;
	void __iomem *regs;
	void __iomem *gpio1_regs;
	void __iomem *gpio2_regs;
	u32 ctrl1_sw_copy;
	bool is_quad;
	u32 pages_per_die;
	int current_die;
#ifndef __UBOOT__
	/* Mutex to protect against concurrent access */
	struct mutex lock;
#endif
};

/* ====================================================================== */
/* CORE DRIVER LOGIC (COMMON)                                         */
/* ====================================================================== */

static int scai_nand_exec_transaction(struct scai_nand_priv *priv,
				      const u8 *tx_buf, u32 tx_len_elems,
				      u8 *rx_buf, u32 rx_len_elems,
				      bool keep_ce);

static int scai_nand_get_feature(struct scai_nand_priv *priv, u8 feature, u8 *value)
{
	const u8 cmd[] = { MT29F_CMD_GET_FEATURES, feature };
	u8 val = 0;
	int ret;

	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	ret = scai_nand_exec_transaction(priv, cmd, sizeof(cmd), &val, sizeof(val), false);
	*value = val;
	return ret;
}

static int scai_nand_set_feature(struct scai_nand_priv *priv, u8 feature, u8 value)
{
	const u8 cmd[] = { MT29F_CMD_SET_FEATURES, feature, value};

	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, false);
}

static int scai_nand_write_enable(struct scai_nand_priv *priv)
{
	const u8 cmd = MT29F_CMD_WRITE_ENABLE;

	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	return scai_nand_exec_transaction(priv, &cmd, sizeof(cmd), NULL, 0, false);
}

static int scai_nand_write_disable(struct scai_nand_priv *priv)
{
	const u8 cmd = MT29F_CMD_WRITE_DISABLE;

	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	return scai_nand_exec_transaction(priv, &cmd, sizeof(cmd), NULL, 0, false);
}

static int scai_nand_wait_flash_ready(struct scai_nand_priv *priv)
{
	u8 status = 0;
	int retries = 1000;
	int err;

	while (--retries) {
		err = scai_nand_get_feature(priv, MT29F_REG_STATUS, &status);
		if (err)
			return -EIO;
		if (!(status & STATUS_OIP_BIT))
			return 0;
		udelay(150);
	}

	dev_err(priv->mtd.dev, "Flash wait ready timeout\n");
	return -ETIMEDOUT;
}

static int scai_nand_select_die(struct scai_nand_priv *priv, int die)
{
	if (die < 0 || die > 1)
		return -EINVAL;

	if (priv->current_die == die)
		return 0;

	u8 die_val = (die == 1) ? MT29F_DIE_1 : MT29F_DIE_0;
	int ret = scai_nand_set_feature(priv, MT29F_REG_DIE_SELECT, die_val);
	if (ret == 0)
		priv->current_die = die;

	return ret;
}

static int scai_nand_unlock_all_blocks(struct scai_nand_priv *priv)
{
	return scai_nand_set_feature(priv, MT29F_REG_LOCK, MT29F_UNLOCK_ALL);
}

static int scai_nand_reset_device(struct scai_nand_priv *priv)
{
	const u8 cmd = MT29F_CMD_RESET_DEVICE;
	int ret;

	ret = scai_nand_exec_transaction(priv, &cmd, sizeof(cmd), NULL, 0, false);
	if (ret)
		return ret;

	priv->current_die = 0;
	return scai_nand_wait_flash_ready(priv);
}

static int scai_read_id(struct scai_nand_priv *priv, u8 *jedec_ids, u32 len)
{
	const u8 cmd[] = { MT29F_CMD_READ_ID, 0xFF };
	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), jedec_ids, len, false);
}

static int scai_nand_init_device(struct scai_nand_priv *priv)
{
	int ret;	

	u8 config_reg = 0;
	ret = scai_nand_get_feature(priv, MT29F_REG_CONFIG, &config_reg);
	if (ret)
		return ret;

	config_reg |= CONFIG_CONTINUOUS;
	ret = scai_nand_set_feature(priv, MT29F_REG_CONFIG, config_reg);
	if (ret)
		return ret;

	ret = scai_nand_unlock_all_blocks(priv);
	if (ret)
		return ret;

	return ret;
}

static u32 scai_nand_fifo_write(struct scai_nand_priv *priv,
				const void* tx_buffer,
				u32 tx_len)
{
	u32 status2_word     = 0;
	u32 elements_written = 0;
	bool  is_word        = (priv->ctrl1_sw_copy & CTRL1_DATA_MODE_WORD) != 0;

	const u8* buf8       = (const u8*) tx_buffer;
	const u32* buf32     = (const u32*)tx_buffer;

	while (elements_written < tx_len) {
		u32 timeout_counter = SCAI_NAND_FIFO_TIMEOUT;

		do {
			status2_word = readl(priv->regs + SCAI_QSPI_REG_STATUS2);
			if (!(status2_word & STATUS2_TX_FIFO_FULL)) {
				break;
			}
			timeout_counter--;
		} while (timeout_counter > 0);

		if (timeout_counter == 0) {
			dev_err(priv->mtd.dev, "Tx FIFO timeout\n");
			return elements_written;
		}

		u32 fifo_wr_cnt = (status2_word & STATUS2_TX_FIFO_WRCNT_MASK) >>
				  STATUS2_TX_FIFO_WRCNT_SHIFT;
		u32 free_space_words = SCAI_NAND_FIFO_LENGTH - fifo_wr_cnt;
		u32 chunk_size = tx_len - elements_written;
		if (chunk_size > free_space_words) {
			chunk_size = free_space_words;
		}

		for (u32 i = 0; i < chunk_size; ++i) {
			u32 data_to_write = 0;

			if (is_word) {
				data_to_write = buf32[elements_written];
			} else {
				data_to_write = (((u32)(buf8[elements_written])) <<
						 SCAI_QSPI_FIFO_BYTE_SHIFT) &
						SCAI_QSPI_FIFO_TX_BYTE_MASK;
				data_to_write |= ~SCAI_QSPI_FIFO_TX_BYTE_MASK;
			}
			
			writel(data_to_write, priv->regs + SCAI_QSPI_REG_DATA);
			elements_written++;
		}
	}

	return elements_written;
}

static u32 scai_nand_fifo_read(struct scai_nand_priv *priv,
			       void* rx_buffer,
			       u32 rx_len)
{
	u32  status2_word  = 0;
	u32  elements_read = 0;
	bool is_word       = (priv->ctrl1_sw_copy & CTRL1_DATA_MODE_WORD) != 0;

	u8* buf8           = (u8*)  rx_buffer;
	u32* buf32         = (u32*) rx_buffer;
	
	bool is_dummy_read = (rx_buffer == NULL);

	while (elements_read < rx_len) {
		u32 timeout_counter = SCAI_NAND_FIFO_TIMEOUT;

		do {
			status2_word = readl(priv->regs + SCAI_QSPI_REG_STATUS2);
			if (!(status2_word & STATUS2_RX_FIFO_EMPTY)) {
				break;
			}
			timeout_counter--;
		} while (timeout_counter > 0);

		if (timeout_counter == 0) {
			dev_err(priv->mtd.dev, "Rx FIFO timeout.\n");
			return elements_read;
		}

		u32 words_available = (status2_word >> STATUS2_RX_FIFO_RDCNT_SHIFT) & STATUS2_RX_FIFO_RDCNT_MASK;
		u32 chunk_size = rx_len - elements_read;
		if (chunk_size > words_available) {
			chunk_size = words_available;
		}

		for (u32 i = 0; i < chunk_size; ++i) {
			u32 value = readl(priv->regs + SCAI_QSPI_REG_DATA);

			if (is_dummy_read) {
				/* Discard the value */
			} else if (is_word) {
				buf32[elements_read] = value;
			} else {
				buf8[elements_read] = (u8)(value & SCAI_QSPI_FIFO_RX_BYTE_MASK);
			}
			elements_read++;
		}
	}

	return elements_read;
}

static int scai_nand_start_transaction(struct scai_nand_priv *priv, u32 tx_len_elems, u32 rx_len_elems)
{
	u32 ctrl1 = priv->ctrl1_sw_copy;

	ctrl1 &= ~(CTRL1_TX_COUNT(0x7FF) | CTRL1_RX_COUNT(0x7FF));
	ctrl1 |= CTRL1_TX_COUNT(tx_len_elems) | CTRL1_RX_COUNT(rx_len_elems);
	ctrl1 |= CTRL1_START;
	writel(ctrl1, priv->regs + SCAI_QSPI_REG_CTRL1);

	ctrl1 |= CTRL1_CHIP_ENABLE;
	writel(ctrl1, priv->regs + SCAI_QSPI_REG_CTRL1);
	priv->ctrl1_sw_copy = ctrl1;

	return 0;
}

static void scai_nand_finish_transaction(struct scai_nand_priv *priv, bool keep_ce)
{
	u32 ctrl1 = priv->ctrl1_sw_copy;

	ctrl1 &= ~(CTRL1_START | CTRL1_TX_COUNT(0x7FF) | CTRL1_RX_COUNT(0x7FF));
	writel(ctrl1, priv->regs + SCAI_QSPI_REG_CTRL1);

	if (!keep_ce) {
		ctrl1 &= ~CTRL1_CHIP_ENABLE;
		writel(ctrl1, priv->regs + SCAI_QSPI_REG_CTRL1);
	}
	priv->ctrl1_sw_copy = ctrl1;
}

static int scai_nand_wait_idle(struct scai_nand_priv *priv)
{
	u32 status;
	u32 retries = SCAI_NAND_FIFO_TIMEOUT;
	u32 buf[4] = {0};

	do {
		status = readl(priv->regs + SCAI_QSPI_REG_STATUS1);
		if (status & STATUS1_IDLE) {
			return 0; /* Success */
		}
		scai_nand_fifo_write(priv, buf, sizeof(buf));
		ndelay(10);
		retries--;
	} while (retries > 0);

	dev_err(priv->mtd.dev, "QSPI controller idle wait timeout\n");
	return -ETIMEDOUT;
}


static int scai_nand_exec_transaction(struct scai_nand_priv *priv,
				      const u8 *tx_buf, u32 tx_len_elems,
				      u8 *rx_buf, u32 rx_len_elems,
				      bool keep_ce)
{
	int ret = 0;	

	scai_nand_start_transaction(priv, tx_len_elems, rx_len_elems);

	if (tx_len_elems > 0) {
		u32 written = scai_nand_fifo_write(priv, tx_buf, tx_len_elems);

		if (written < tx_len_elems) {
			dev_err(priv->mtd.dev, "FIFO write failed (wrote %u of %u)\n",
				written, tx_len_elems);
				scai_nand_finish_transaction(priv, keep_ce);			
				return -ETIMEDOUT;
		}
	}

	if (rx_len_elems > 0) {
		u32 read_count = scai_nand_fifo_read(priv, rx_buf, rx_len_elems);

		if (read_count < rx_len_elems) {
			dev_err(priv->mtd.dev, "FIFO read failed (read %u of %u)\n",
				read_count, rx_len_elems);
			scai_nand_finish_transaction(priv, keep_ce);
			return -ETIMEDOUT;
		}
	}

	ret = scai_nand_wait_idle(priv);
	if (ret)
		return ret;

	scai_nand_finish_transaction(priv, keep_ce);
	return 0;
}

static int scai_nand_page_read_to_cache(struct scai_nand_priv *priv, int page_addr)
{
	u8 cmd[4];

	cmd[0] = MT29F_CMD_PAGE_READ_TO_CACHE;
	cmd[1] = (page_addr >> 16) & 0xFF;
	cmd[2] = (page_addr >> 8) & 0xFF;
	cmd[3] = page_addr & 0xFF;

	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, false);
}

static int scai_nand_read_from_cache(struct scai_nand_priv *priv, u16 col, u8 *buf,
				     u32 len_bytes, bool use_word_mode)
{
	u8 cmd[4];
	int ret;
	u32 rx_elements = (use_word_mode ? ((len_bytes + 3) / 4) : len_bytes);

	dev_err(priv->mtd.dev, "Read from cache: word_mode = %d, quad mode = %d\n",
		use_word_mode, priv->is_quad);

	cmd[0] = priv->is_quad ? MT29F_CMD_READ_FROM_CACHE_X4 : MT29F_CMD_READ_FROM_CACHE_X1;
	cmd[1] = (col >> 8) & 0xFF;
	cmd[2] = col & 0xFF;
	cmd[3] = 0x00; /* dummy byte */

	/* Phase 1: Send command (x1, Byte mode), keep CE active */
	dev_err(priv->mtd.dev, "Phase 1: send command\n");
	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);
	ret = scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, true);
	if (ret)
		return ret;

	/*
	 * Phase 2: Dummy Read (HSS SCAI Quirk)
	 */
	if (priv->is_quad) {
		u32 dummy_rx_len_words = (col >> 2);
		priv->ctrl1_sw_copy |= (CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

		dev_err(priv->mtd.dev, "Phase 2 - dummy read %u words\n", dummy_rx_len_words);
		ret = scai_nand_exec_transaction(priv, NULL, 0,
					   buf, dummy_rx_len_words,
					   true);
		if (ret)
			return ret;

		for (u32 i = 0; i < dummy_rx_len_words; ++i) {
			dev_err(priv->mtd.dev, "buf[%u] = 0x%08X (dummy)\n", i, ((u32*)buf)[i]);
		}
	}

	/* Phase 3: Real Read (Data phase) */
	if (!priv->is_quad) {
		if (use_word_mode)
			priv->ctrl1_sw_copy |= CTRL1_DATA_MODE_WORD;
		else
			priv->ctrl1_sw_copy &= ~CTRL1_DATA_MODE_WORD;
	}

	dev_err(priv->mtd.dev, "Phase 3 - real read %d words\n", rx_elements);
	return scai_nand_exec_transaction(priv, NULL, 0, buf, rx_elements, false);
}

static int scai_nand_program_load(struct scai_nand_priv *priv, u16 col, const u8 *buf,
				  u32 len_bytes, bool use_word_mode)
{
	u8 cmd[3];
	int ret;
	u32 tx_elements = (use_word_mode ? ((len_bytes + 3) / 4) : len_bytes);
	dev_err(priv->mtd.dev, "Program load: word_mode = %d, quad mode = %d, tx_elements=%d\n",
		use_word_mode, priv->is_quad, tx_elements);

	cmd[0] = priv->is_quad ? MT29F_CMD_PROGRAM_LOAD_X4 : MT29F_CMD_PROGRAM_LOAD_X1;
	cmd[1] = (u8)((col >> 8) & 0xFF);
	cmd[2] = (u8)(col & 0xFF);

	/* Send command (x1, Byte mode), keep CE active */
	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	ret = scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, true);
	if (ret) {
		return ret;
	}
	
	if (priv->is_quad) {
		priv->ctrl1_sw_copy |= (CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);
	}
	
	/* Send data for programming, release CE */
	return scai_nand_exec_transaction(priv, buf, tx_elements, NULL, 0, false);
}

static int scai_nand_program_execute(struct scai_nand_priv *priv, int page_addr)
{
	u8 cmd[4];

	cmd[0] = MT29F_CMD_PROGRAM_EXECUTE;
	cmd[1] = (page_addr >> 16) & 0xFF;
	cmd[2] = (page_addr >> 8) & 0xFF;
	cmd[3] = page_addr & 0xFF;

	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);
	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, false);
}

static int scai_nand_block_erase(struct scai_nand_priv *priv, int page_addr)
{
	u8 cmd[4];

	cmd[0] = MT29F_CMD_BLOCK_ERASE;
	cmd[1] = (page_addr >> 16) & 0xFF;
	cmd[2] = (page_addr >> 8) & 0xFF;
	cmd[3] = page_addr & 0xFF;

	dev_err(priv->mtd.dev, "Erasing block at page address: 0x%06X\n", page_addr);

	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	return scai_nand_exec_transaction(priv, cmd, sizeof(cmd), NULL, 0, false);
}


static void scai_nand_set_power(struct scai_nand_priv *priv, bool enable)
{
	u32 val1, val2;

	if (!priv->gpio1_regs || !priv->gpio2_regs) {
#ifdef __UBOOT__
		printf("WARN: SCAI NAND: GPIO registers not mapped\n");
#else
		dev_warn(priv->mtd.dev, "GPIO registers not mapped\n");
#endif
		return;
	}

	val1 = 0;
	val2 = 0;

	if (enable) {
		val1 |= GPIO1_ENA_SS1_MASK;
		val2 |= GPIO2_ENA_SS2_MASK;
	} else {
		val1 &= ~GPIO1_ENA_SS1_MASK;
		val2 &= ~GPIO2_ENA_SS2_MASK;
	}

	writel(val1, priv->gpio1_regs + GPIO_REG_WDATA_OFFSET);
	writel(val2, priv->gpio2_regs + GPIO_REG_WDATA_OFFSET);
}

static int scai_nand_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
#ifndef __UBOOT__
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct scai_nand_priv *priv = container_of(nand, struct scai_nand_priv, nand);
#endif
	int ret;

	dev_err(mtd->dev, "Debug: scai_nand_mtd_erase() called.\n");

#ifndef __UBOOT__
	mutex_lock(&priv->lock);
#endif

	ret = nanddev_mtd_erase(mtd, instr);

#ifndef __UBOOT__
	mutex_unlock(&priv->lock);
#endif
	return ret;
}

static int scai_nand_op_erase(struct nand_device *nand,
			    const struct nand_pos *pos)
{
	struct scai_nand_priv *priv = container_of(nand, struct scai_nand_priv, nand);
	int ret;
	int row = nanddev_pos_to_row(nand, pos);

	dev_err(priv->mtd.dev, "Erasing row %d\n", row);

	dev_err(priv->mtd.dev, "Set DIE\n");
	ret = scai_nand_select_die(priv, pos->target);
	if (ret)
		return ret;

	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	ret = scai_nand_write_enable(priv);
	if (ret)
		return ret;

	ret = scai_nand_block_erase(priv, row);
	if (ret)
		return ret;

	return scai_nand_wait_flash_ready(priv);
}

static int scai_nand_mtd_block_isbad(struct mtd_info *mtd, loff_t offs)
{
	struct nand_device *nand = mtd_to_nanddev(mtd);
#ifndef __UBOOT__
	struct scai_nand_priv *priv = container_of(nand, struct scai_nand_priv, nand);
#endif
	struct nand_pos pos;
	int ret;

	dev_err(mtd->dev, "Debug: scai_nand_mtd_block_isbad() called.\n");

#ifndef __UBOOT__
	mutex_lock(&priv->lock);
#endif

	nanddev_offs_to_pos(nand, offs, &pos);
	dev_err(mtd->dev, "Debug: scai_nand_mtd_block_isbad() exiting.\n");
	ret = nand->ops->isbad(nand, &pos);

#ifndef __UBOOT__
	mutex_unlock(&priv->lock);
#endif
	return ret;
}

static bool scai_nand_op_isbad(struct nand_device *nand,
			     const struct nand_pos *pos)
{
	dev_err(nand->mtd->dev, "Debug: scai_nand_op_isbad() called.\n");
	return false;
}

static int scai_nand_mtd_block_markbad(struct mtd_info *mtd, loff_t offs)
{
	struct nand_device *nand = mtd_to_nanddev(mtd);
#ifndef __UBOOT__
	struct scai_nand_priv *priv = container_of(nand, struct scai_nand_priv, nand);
#endif
	struct nand_pos pos;
	int ret;

	dev_err(mtd->dev, "Debug: scai_nand_mtd_block_markbad() called.\n");

#ifndef __UBOOT__
	mutex_lock(&priv->lock);
#endif

	nanddev_offs_to_pos(nand, offs, &pos);
	ret = nand->ops->markbad(nand, &pos);

#ifndef __UBOOT__
	mutex_unlock(&priv->lock);
#endif
	return ret;
}

static int scai_nand_op_markbad(struct nand_device *nand,
			      const struct nand_pos *pos)
{
	return -EOPNOTSUPP;
}

static int scai_nand_mtd_block_isreserved(struct mtd_info *mtd, loff_t offs)
{
#ifndef __UBOOT__
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct scai_nand_priv *priv = container_of(nand, struct scai_nand_priv, nand);
#endif
	int ret;

	dev_err(mtd->dev, "Debug: scai_nand_mtd_block_isreserved() called.\n");

#ifndef __UBOOT__
	mutex_lock(&priv->lock);
#endif

	ret = 0;

#ifndef __UBOOT__
	mutex_unlock(&priv->lock);
#endif
	return ret;
}


static const struct nand_ops scai_nand_ops = {
	.erase = scai_nand_op_erase,
	.isbad = scai_nand_op_isbad,
	.markbad = scai_nand_op_markbad,
};

static int scai_nand_mtd_read_oob(struct mtd_info *mtd, loff_t from,
				  struct mtd_oob_ops *ops)
{
	struct nand_device *nand    = mtd_to_nanddev(mtd);
	struct scai_nand_priv *priv = container_of(nand, struct scai_nand_priv, nand);
	struct nand_io_iter iter;
	int ret = 0;
	bool use_word_mode_data = priv->is_quad && ((nand->memorg.pagesize % 4) == 0);

#ifndef __UBOOT__
	mutex_lock(&priv->lock);
#endif

	dev_err(mtd->dev, "Debug: scai_nand_mtd_read_oob() called.\n");

	nanddev_io_for_each_page(nand, from, ops, &iter) {
		const struct nand_pos *pos = &iter.req.pos;
		int row = nanddev_pos_to_row(nand, pos);

		dev_err(mtd->dev, "Debug: Read loop start (Row: %d, Target: %d)\n",
			row, pos->target);

		dev_err(mtd->dev, "Debug: Read selecting die...\n");
		ret = scai_nand_select_die(priv, pos->target);
		if (ret)
			break;

		dev_err(mtd->dev, "Debug: Read (A) calling page_read_to_cache...\n");
		ret = scai_nand_page_read_to_cache(priv, row);
		if (ret)
			break;

		dev_err(mtd->dev, "Debug: Read (B) calling wait_flash_ready...\n");
		ret = scai_nand_wait_flash_ready(priv);
		if (ret)
			break;

		if (iter.req.datalen) {
			dev_err(mtd->dev, "Debug: Read (C) calling read_from_cache (data)...\n");
			ret = scai_nand_read_from_cache(priv, iter.req.dataoffs,
							iter.req.databuf.in,
							iter.req.datalen,
							use_word_mode_data);
			if (ret)
				break;
		}

		if (iter.req.ooblen) {
			u16 col = nand->memorg.pagesize + iter.req.ooboffs;
			dev_err(mtd->dev, "Debug: Read (D) calling read_from_cache (oob)...\n");
			ret = scai_nand_read_from_cache(priv, col,
							iter.req.oobbuf.in,
							iter.req.ooblen,
							false); /* Force byte mode for OOB */
			if (ret)
				break;
		}
	}

	ops->retlen = ops->len - iter.dataleft;
	ops->oobretlen = ops->ooblen - iter.oobleft;
	dev_err(mtd->dev, "Debug: scai_nand_mtd_read_oob() exiting.\n");

#ifndef __UBOOT__
	mutex_unlock(&priv->lock);
#endif
	return ret;
}

static int scai_nand_mtd_write_oob(struct mtd_info *mtd, loff_t to,
				   struct mtd_oob_ops *ops)
{
	struct nand_device *nand    = mtd_to_nanddev(mtd);
	struct scai_nand_priv *priv = container_of(nand, struct scai_nand_priv, nand);
	struct nand_io_iter iter;
	int ret = 0;
	bool use_word_mode_data = priv->is_quad && ((nand->memorg.pagesize % 4) == 0);

#ifndef __UBOOT__
	mutex_lock(&priv->lock);
#endif

	dev_err(mtd->dev, "Write: scai_nand_mtd_write_oob() called, use_word_mode_data=%d\n",
		use_word_mode_data);

	priv->ctrl1_sw_copy |= CTRL1_NWP;
	writel(priv->ctrl1_sw_copy, priv->regs + SCAI_QSPI_REG_CTRL1);

	nanddev_io_for_each_page(nand, to, ops, &iter) {
		const struct nand_pos *pos = &iter.req.pos;
		int row = nanddev_pos_to_row(nand, pos);
		
		ret = scai_nand_select_die(priv, pos->target);
		if (ret) {
			break;
		}

		dev_err(mtd->dev, "Write: Load data, row = %d\n", row);

		if (iter.req.datalen) {
			dev_err(mtd->dev, "Write: Load data - we\n");
			ret = scai_nand_write_enable(priv);
			if (ret) {
				break;
			}
			dev_err(mtd->dev, "Write: len=%d, col=%d\n",
				iter.req.datalen, iter.req.dataoffs);
			ret = scai_nand_program_load(priv, iter.req.dataoffs,
						     iter.req.databuf.out,
						     iter.req.datalen,
						     use_word_mode_data);
			if (ret) {
				break;
			}
		}
		
		dev_err(mtd->dev, "Write: Load OOB data\n");

		if (iter.req.ooblen) {
			u16 col = nand->memorg.pagesize + iter.req.ooboffs;
			ret = scai_nand_write_enable(priv);
			if (ret) {
				break;
			}
			ret = scai_nand_program_load(priv, col,
						     iter.req.oobbuf.out,
						     iter.req.ooblen,
						     false); /* Force byte mode for OOB */
			if (ret) {
				break;
			}
		}

		dev_err(mtd->dev, "Write: WE\n");
		ret = scai_nand_write_enable(priv);
		if (ret) {
			break;
		}

		dev_err(mtd->dev, "Write: Execute programming\n");
		ret = scai_nand_program_execute(priv, row);
		if (ret) {
			break;
		}

		dev_err(mtd->dev, "Write: Wait OiP\n");
		ret = scai_nand_wait_flash_ready(priv);
		if (ret) {
			break;
		}
	}
	
	scai_nand_write_disable(priv);

	priv->ctrl1_sw_copy &= ~CTRL1_NWP;	
	writel(priv->ctrl1_sw_copy, priv->regs + SCAI_QSPI_REG_CTRL1);

	ops->retlen = ops->len - iter.dataleft;
	ops->oobretlen = ops->ooblen - iter.oobleft;

#ifndef __UBOOT__
	mutex_unlock(&priv->lock);
#endif
	return ret;
}


/* ====================================================================== */
/* U-BOOT DRIVER MODEL (DM) PROBE/REMOVE                              */
/* ====================================================================== */
#ifdef __UBOOT__

static int scai_nand_probe(struct udevice *dev)
{
	struct scai_nand_priv *priv = dev_get_priv(dev);
	struct nand_device *nand = &priv->nand;
	struct mtd_info *mtd  = &priv->mtd;
	u8 jedec_ids[2];
	int ret;

	priv->regs = dev_remap_addr_index(dev, 0);
	if (!priv->regs) {
		dev_err(dev, "Failed to map QSPI registers\n");
		return -EINVAL;
	}
	dev_err(dev, "QSPI_REG mapped to VA: %p\n", priv->regs);

	priv->gpio1_regs = dev_remap_addr_index(dev, 1);
	if (!priv->gpio1_regs) {
		dev_err(dev, "Failed to map GPIO_1 registers\n");
		return -EINVAL;
	}
	dev_err(dev, "GPIO_1 mapped to VA: %p, Value: 0x%08X\n",
             priv->gpio1_regs, readl(priv->gpio1_regs + GPIO_REG_RDATA_OFFSET));

	priv->gpio2_regs = dev_remap_addr_index(dev, 2);
	if (!priv->gpio2_regs) {
		dev_err(dev, "Failed to map GPIO_2 registers\n");
		return -EINVAL;
	}
	dev_err(dev, "GPIO_2 mapped to VA: %p, Value: 0x%08X\n",
             priv->gpio2_regs, readl(priv->gpio2_regs + GPIO_REG_RDATA_OFFSET));

	scai_nand_set_power(priv, true);
	dev_err(dev, "Enabled MT29F power via custom GPIOs\n");

	nand->mtd = mtd;
	mtd->priv = nand;
	mtd->dev = dev;
	mtd->name = (char *)dev->name;
	priv->current_die = -1;

	priv->is_quad = dev_read_bool(dev, "spi-tx-bus-width-4");
	if (priv->is_quad) {
		dev_err(dev, "Quad mode selected via device tree.\n");
	} else {
		dev_err(dev, "Single (x1) mode selected (default).\n");
	}

	priv->ctrl1_sw_copy = CTRL1_RESET;
	priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);
	writel(priv->ctrl1_sw_copy, priv->regs + SCAI_QSPI_REG_CTRL1);

	ret = scai_nand_reset_device(priv);
	if (ret) {
		dev_err(dev, "Failed to reset device on probe\n");
		goto err_power_off;
	}

	ret = scai_nand_init_device(priv);
	if (ret) {
		dev_err(dev, "Failed to initialize device on probe\n");
		goto err_power_off;
	}

	ret = scai_read_id(priv, jedec_ids, sizeof(jedec_ids));
	if (ret)
		goto err_power_off;

	dev_err(dev, "JEDEC ID: %02X %02X\n",
		 jedec_ids[0], jedec_ids[1]);

	if (jedec_ids[0] != MT29F_JEDEC_MANUFACTURER_ID ||
	    jedec_ids[1] != MT29F_JEDEC_DEVICE_ID_0) {
		dev_warn(dev, "JEDEC ID mismatch, expected %02X %02X\n",
			 MT29F_JEDEC_MANUFACTURER_ID, MT29F_JEDEC_DEVICE_ID_0);
	}

	nand->memorg = (struct nand_memory_organization) {
		.bits_per_cell = MT29F_BITS_PER_CELL,
		.pagesize = MT29F_PAGE_SIZE,
		.oobsize = MT29F_OOB_SIZE,
		.pages_per_eraseblock = MT29F_PAGES_PER_BLOCK,
		.eraseblocks_per_lun = MT29F_BLOCKS_PER_DIE,
		.luns_per_target = MT29F_LUNS_PER_DIE,
		.ntargets = 1,
		.planes_per_lun = 1,
	};

	nand->eccreq.strength = 0;
	nand->eccreq.step_size = 0;

	ret = nanddev_init(nand, &scai_nand_ops, NULL);
	if (ret) {
		dev_err(dev, "nanddev_init failed: %d\n", ret);
		goto err_power_off;
	}

	mtd->_read = NULL;
	mtd->_write = NULL;
	mtd->_read_oob = scai_nand_mtd_read_oob;
	mtd->_write_oob = scai_nand_mtd_write_oob;
	mtd->_erase = scai_nand_mtd_erase;
	mtd->_block_isbad = scai_nand_mtd_block_isbad;
	mtd->_block_markbad = scai_nand_mtd_block_markbad;
	mtd->_block_isreserved = scai_nand_mtd_block_isreserved;
	mtd->flags = MTD_CAP_NANDFLASH | MTD_WRITEABLE;

	priv->pages_per_die = nand->memorg.pages_per_eraseblock *
			      nand->memorg.eraseblocks_per_lun;
	nand->memorg.ntargets = 2;
	mtd->size = nanddev_size(nand);

	dev_info(dev, "Found 1 die, size %llu. Adjusting for 2 dies.\n",
		 (unsigned long long)mtd->size / 2);

	ret = add_mtd_device(mtd);
	if (ret) {
		dev_err(dev, "add_mtd_device failed: %d\n", ret);
		goto err_nand_cleanup;
	}

	dev_info(dev, "SCAI MT29F driver initialized (Page: %u, OOB: %u, Block: %uKB, Total: %lluMB)\n",
		 mtd->writesize, mtd->oobsize, mtd->erasesize / 1024,
		 (unsigned long long)mtd->size >> 20);
	return 0;

err_nand_cleanup:
	nanddev_cleanup(nand);
err_power_off:
	return ret;
}

static int scai_nand_remove(struct udevice *dev)
{
	struct scai_nand_priv *priv = dev_get_priv(dev);
	struct nand_device *nand = &priv->nand;
	struct mtd_info *mtd = &priv->mtd;
	int ret;

	ret = del_mtd_device(mtd);
	if (ret)
		dev_err(dev, "del_mtd_device failed: %d\n", ret);

	nanddev_cleanup(nand);
	dev_info(dev, "MT29F power left enabled\n");

	return ret;
}

static const struct udevice_id scai_nand_of_match[] = {
	{ .compatible = "navc,scai-qspi-mt29f" },
	{ /* sentinel */ }
};

U_BOOT_DRIVER(mchp_scai_nand) = {
	.name           = "mchp_scai_nand",
	.id             = UCLASS_MTD,
	.of_match       = scai_nand_of_match,
	.probe          = scai_nand_probe,
	.remove         = scai_nand_remove,
	.priv_auto      = sizeof(struct scai_nand_priv),
};

/* ====================================================================== */
/* LINUX KERNEL PLATFORM DRIVER PROBE/REMOVE                          */
/* ====================================================================== */
#else

static int scai_nand_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    struct scai_nand_priv *priv;
    struct nand_device *nand;
    struct mtd_info *mtd;
    u8 jedec_ids[2];
    int ret;

    priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
    if (!priv)
        return -ENOMEM;

    mutex_init(&priv->lock);

    nand = &priv->nand;
    mtd = &priv->mtd;

    mtd->dev.parent = dev;
    nand->mtd = mtd;
    mtd->priv = nand;
    platform_set_drvdata(pdev, priv);

    priv->regs = devm_platform_ioremap_resource(pdev, 0);
    if (IS_ERR(priv->regs)) {
        dev_err(dev, "Failed to map QSPI registers\n");
        return PTR_ERR(priv->regs);
    }
    dev_info(dev, "QSPI_REG mapped to VA: %p\n", priv->regs);

    priv->gpio1_regs = devm_platform_ioremap_resource(pdev, 1);
    if (IS_ERR(priv->gpio1_regs)) {
        dev_err(dev, "Failed to map GPIO_1 registers\n");
        return PTR_ERR(priv->gpio1_regs);
    }
    dev_info(dev, "GPIO_1 mapped to VA: %p\n", priv->gpio1_regs);

    priv->gpio2_regs = devm_platform_ioremap_resource(pdev, 2);
    if (IS_ERR(priv->gpio2_regs)) {
        dev_err(dev, "Failed to map GPIO_2 registers\n");
        return PTR_ERR(priv->gpio2_regs);
    }
    dev_info(dev, "GPIO_2 mapped to VA: %p\n", priv->gpio2_regs);

    scai_nand_set_power(priv, true);
    dev_info(dev, "Enabled MT29F power via custom GPIOs\n");

    mtd->name = devm_kasprintf(dev, GFP_KERNEL, "scai-nand-%s", dev_name(dev));
    if (!mtd->name)
        return -ENOMEM;
    
    priv->current_die = -1;

    priv->is_quad = device_property_read_bool(dev, "spi-tx-bus-width-4");
    if (priv->is_quad) {
        dev_info(dev, "Quad mode selected via device tree.\n");
    } else {
        dev_info(dev, "Single (x1) mode selected (default).\n");
    }

    priv->ctrl1_sw_copy = CTRL1_RESET;
    priv->ctrl1_sw_copy &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);
    writel(priv->ctrl1_sw_copy, priv->regs + SCAI_QSPI_REG_CTRL1);

    ret = scai_nand_reset_device(priv);
    if (ret) {
        dev_err(dev, "Failed to reset device on probe\n");
        goto err_power_off;
    }

    ret = scai_nand_init_device(priv);
    if (ret) {
        dev_err(dev, "Failed to initialize device on probe\n");
        goto err_power_off;
    }

    ret = scai_read_id(priv, jedec_ids, sizeof(jedec_ids));
    if (ret)
        goto err_power_off;

    dev_info(dev, "JEDEC ID: %02X %02X\n", jedec_ids[0], jedec_ids[1]);

    if (jedec_ids[0] != MT29F_JEDEC_MANUFACTURER_ID ||
        jedec_ids[1] != MT29F_JEDEC_DEVICE_ID_0) {
        dev_warn(dev, "JEDEC ID mismatch, expected %02X %02X\n",
                 MT29F_JEDEC_MANUFACTURER_ID, MT29F_JEDEC_DEVICE_ID_0);
    }

    nand->memorg = (struct nand_memory_organization) {
        .bits_per_cell = MT29F_BITS_PER_CELL,
        .pagesize = MT29F_PAGE_SIZE,
        .oobsize = MT29F_OOB_SIZE,
        .pages_per_eraseblock = MT29F_PAGES_PER_BLOCK,
        .eraseblocks_per_lun = MT29F_BLOCKS_PER_DIE,
        .luns_per_target = MT29F_LUNS_PER_DIE,
        .ntargets = 1,
        .planes_per_lun = 1,
    };

    nand->eccreq.strength = 0;
    nand->eccreq.step_size = 0;

    ret = nanddev_init(nand, &scai_nand_ops, THIS_MODULE);
    if (ret) {
        dev_err(dev, "nanddev_init failed: %d\n", ret);
        goto err_power_off;
    }

    mtd->_read = NULL;
    mtd->_write = NULL;
    mtd->_read_oob = scai_nand_mtd_read_oob;
    mtd->_write_oob = scai_nand_mtd_write_oob;
    mtd->_erase = scai_nand_mtd_erase;
    mtd->_block_isbad = scai_nand_mtd_block_isbad;
    mtd->_block_markbad = scai_nand_mtd_block_markbad;
    mtd->_block_isreserved = scai_nand_mtd_block_isreserved;
    mtd->flags = MTD_CAP_NANDFLASH | MTD_WRITEABLE;

    priv->pages_per_die = nand->memorg.pages_per_eraseblock *
                          nand->memorg.eraseblocks_per_lun;
    nand->memorg.ntargets = 2; 
    mtd->size = nanddev_size(nand); 

    dev_info(dev, "Found 1 die, size %llu. Adjusting for 2 dies.\n",
           (unsigned long long)mtd->size / 2);

    ret = mtd_device_register(mtd, NULL, 0);
    if (ret) {
        dev_err(dev, "mtd_device_register failed: %d\n", ret);
        goto err_nand_cleanup;
    }

    dev_info(dev, "SCAI MT29F driver initialized (Page: %u, OOB: %u, Block: %uKB, Total: %lluMB)\n",
           mtd->writesize, mtd->oobsize, mtd->erasesize / 1024,
           (unsigned long long)mtd->size >> 20);
    return 0;

err_nand_cleanup:
    nanddev_cleanup(nand);
err_power_off:
    scai_nand_set_power(priv, false);
    return ret;
}

static int scai_nand_remove(struct platform_device *pdev)
{
	struct scai_nand_priv *priv = platform_get_drvdata(pdev);
	struct nand_device *nand = &priv->nand;
	struct mtd_info *mtd = &priv->mtd;
	int ret;

	ret = mtd_device_unregister(mtd);
	if (ret)
		dev_err(&pdev->dev, "mtd_device_unregister failed: %d\n", ret);

	nanddev_cleanup(nand);
	scai_nand_set_power(priv, false);
	dev_info(&pdev->dev, "SCAI MT29F driver removed, power disabled.\n");

	return ret;
}

static const struct of_device_id scai_nand_of_match[] = {
	{ .compatible = "navc,scai-qspi-mt29f" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, scai_nand_of_match);

static struct platform_driver mchp_scai_nand_driver = {
	.driver = {
		.name           = "mchp_scai_nand",
		.of_match_table = scai_nand_of_match,
	},
	.probe          = scai_nand_probe,
	.remove         = scai_nand_remove,
};
module_platform_driver(mchp_scai_nand_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("SCAI QSPI MT29F NAND Driver");

#endif /* __UBOOT__ */