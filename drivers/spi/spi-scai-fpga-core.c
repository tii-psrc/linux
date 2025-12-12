// SPDX-License-Identifier: GPL-2.0
/*
 * scai_fpgaqspi.c
 *
 * SCAI FPGA QSPI controller.
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>

#define SCAI_NAND_FIFO_TIMEOUT 100
#define SCAI_NAND_FIFO_LENGTH  64

// Constants for packing a byte into a 32-bit word for the hardware.
// This is required if the hardware expects the byte in the MSB position.
#define SCAI_QSPI_FIFO_BYTE_SHIFT   24
#define SCAI_QSPI_FIFO_TX_BYTE_MASK 0xFF000000
#define SCAI_QSPI_FIFO_RX_BYTE_MASK 0x000000FF

/* --- SCAI QSPI Controller Register Offsets --- */
#define SCAI_QSPI_REG_DATA          0x00
#define SCAI_QSPI_REG_CTRL1         0x04
#define SCAI_QSPI_REG_CTRL2         0x08
#define SCAI_QSPI_REG_CTRL3         0x0C

#define SCAI_QSPI_REG_STATUS1       0x04
#define SCAI_QSPI_REG_STATUS2       0x08

/* --- SCAI QSPI Controller CTRL1 Register Bits --- */
#define CTRL1_CHIP_ENABLE           BIT(0)
#define CTRL1_NWP                   BIT(1)
#define CTRL1_RESET                 BIT(2)
#define CTRL1_DATA_MODE_WORD        BIT(3) /* 0 = Byte, 1 = Word */
#define CTRL1_LANE_WIDTH_X4         BIT(4) /* 0 = x1, 1 = x4 */
#define CTRL1_START                 BIT(9)
/* Count in bytes or words depending on CTRL1_DATA_MODE_WORD */
#define CTRL1_TX_COUNT(n)           (((n) & 0x7FF) << 10)
/* Count in bytes or words depending on CTRL1_DATA_MODE_WORD */
#define CTRL1_RX_COUNT(n)           (((n) & 0x7FF) << 21)

/* --- SCAI QSPI Controller Status2 Register Bits --- */
#define STATUS2_RX_FIFO_FULL         BIT(0)
#define STATUS2_RX_FIFO_EMPTY		 BIT(1)
#define STATUS2_RX_FIFO_RDCNT_MASK   0x7F
#define STATUS2_RX_FIFO_RDCNT_SHIFT  2
#define STATUS2_RX_FIFO_WrCnt_MASK   0x7F
#define STATUS2_RX_FIFO_WrCnt_SHIFT  9
#define STATUS2_TX_FIFO_FULL         BIT(16)
#define STATUS2_TX_FIFO_EMPTY        BIT(17)
#define STATUS2_TX_FIFO_RDCNT_MASK   0x7F
#define STATUS2_TX_FIFO_RDCNT_SHIFT  18
#define STATUS2_TX_FIFO_WRCNT_MASK   0x7F
#define STATUS2_TX_FIFO_WRCNT_SHIFT  25

/* --- SCAI QSPI Controller STATUS1 Register Bits --- */
#define STATUS1_IDLE                BIT(0)


/*
 * GPIO definitions based on scai_fpga_platform.h from HSS.
 */
#define GPIO_REG_WDATA_OFFSET   0x00
#define GPIO_REG_RDATA_OFFSET   0x04
#define GPIO1_ENA_SS1_MASK      BIT(4)
#define GPIO2_ENA_SS2_MASK      BIT(0)

#define TIMEOUT_MS             (1000 * 500)

#define MAX_DATA_CMD_LEN       0x440

/**
 * struct scai_fpgaqspi_priv - Private driver data structure
 */
struct scai_fpgaqspi_priv {
	/* Register base addresses */
	void __iomem	*base_regs;          /* QSPI register base */
	void __iomem	*gpio1_regs;    /* GPIO1 register base */
	void __iomem	*gpio2_regs;    /* GPIO2 register base */

	/* Software-maintained copies */
	u32		 ctrl1_sw_copy;  /* Cached CTRL1 register value */

	/* Transfer buffers */
	u8		*tx_buf;        /* TX buffer pointer */
	u8		*rx_buf;        /* RX buffer pointer */
	int		 tx_len;        /* TX length */
	int		 rx_len;        /* RX length */

	struct completion transfer_completion;
	struct mutex op_lock; /* lock access to the device */
};

static int scai_fpgaqspi_wait_for_ready(struct scai_fpgaqspi_priv *p )
{
	unsigned long count = 0;
	u32 status;

	do {
		status = readl(p->base_regs + SCAI_QSPI_REG_STATUS1);
		if (status & STATUS1_IDLE)
			return 0;

		udelay(1);
		count++;
	} while (count < TIMEOUT_MS);

	printk("%s: timeout 0x%08X\n", __func__, status);
	return -ETIMEDOUT;
}

static void scai_fpgaqspi_set_operate_mode(struct scai_fpgaqspi_priv *p,
		bool word)
{
	u32 ctrl = p->ctrl1_sw_copy;
	ctrl &= ~(CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	if (word)
		ctrl |= (CTRL1_LANE_WIDTH_X4 | CTRL1_DATA_MODE_WORD);

	p->ctrl1_sw_copy = ctrl;
}

static int scai_fpgaqspi_write_op(struct scai_fpgaqspi_priv *p, bool word)
{
	u32 data, status;
	int err = 0;

	if (word) {
		while (p->tx_len > 0) {
			do {
				status = readl(p->base_regs + SCAI_QSPI_REG_STATUS2);
			} while (status & STATUS2_TX_FIFO_FULL);

			data = *(u32 *)p->tx_buf;
			p->tx_buf += 4;
			p->tx_len -= 4;
#if 0
			printk("%s-word: data(0x%08X)\n", __func__, data);
#endif
			writel(data, p->base_regs + SCAI_QSPI_REG_DATA);
		}
	} else {
		while (p->tx_len--) {
			do {
				status = readl(p->base_regs + SCAI_QSPI_REG_STATUS2);
			} while (status & STATUS2_TX_FIFO_FULL);

			data =  (u32)((*p->tx_buf <<
						SCAI_QSPI_FIFO_BYTE_SHIFT) & SCAI_QSPI_FIFO_TX_BYTE_MASK);
			data |= ~SCAI_QSPI_FIFO_TX_BYTE_MASK;

#if 0
			printk("%s-byte: data(0x%08X)\n", __func__, data);
#endif
			writel(data, p->base_regs + SCAI_QSPI_REG_DATA);
			p->tx_buf++;
		}
	}

	return err;
}

static int scai_fpgaqspi_read_op(struct scai_fpgaqspi_priv *p, bool word)
{
	u32 data, status;

	if (!p->rx_len)
		return -1;

	if (word) {

		while (p->rx_len > 0) {
			do {
				status = readl(p->base_regs + SCAI_QSPI_REG_STATUS2);
			} while (status & STATUS2_RX_FIFO_EMPTY);

			data = readl(p->base_regs + SCAI_QSPI_REG_DATA);
#if 0
			printk("%s-word: data(0x%08X)\n", __func__, data);
#endif
			*(u32 *)p->rx_buf = data;
			p->rx_buf += 4;
			p->rx_len -= 4;
		}
	} else {
		while (p->rx_len--) {
			do {
				status = readl(p->base_regs + SCAI_QSPI_REG_STATUS2);
			} while (status & STATUS2_RX_FIFO_EMPTY);

			data = readl(p->base_regs + SCAI_QSPI_REG_DATA);
#if 0
			printk("%s-byte: data(0x%08X)\n", __func__, data);
#endif
			*p->rx_buf++ = (data & SCAI_QSPI_FIFO_RX_BYTE_MASK);
		}
	}

	return 0;
}

static int scai_fpgaqspi_start_transaction(struct scai_fpgaqspi_priv *p,
		u32 tx_len, u32 rx_len)
{
	u32 ctrl = p->ctrl1_sw_copy;
	ctrl &= ~(CTRL1_TX_COUNT(0x7FF) | CTRL1_RX_COUNT(0x7FF));
	ctrl |= CTRL1_TX_COUNT(tx_len) | CTRL1_RX_COUNT(rx_len);
	ctrl |= CTRL1_START | CTRL1_CHIP_ENABLE;

	writel(ctrl, p->base_regs + SCAI_QSPI_REG_CTRL1);
	p->ctrl1_sw_copy = ctrl;
	return 0;
}

static void scai_fpgaqspi_finish_transaction(struct scai_fpgaqspi_priv *p,
		bool keep_ce)
{
	u32 ctrl1 = p->ctrl1_sw_copy;

	ctrl1 &= ~(CTRL1_START | CTRL1_TX_COUNT(0x7FF) | CTRL1_RX_COUNT(0x7FF));

	if (!keep_ce) {
		ctrl1 &= ~CTRL1_CHIP_ENABLE;
	}

	writel(ctrl1, p->base_regs + SCAI_QSPI_REG_CTRL1);
	p->ctrl1_sw_copy = ctrl1;
}

static int __do_exec_word_op(struct scai_fpgaqspi_priv *p,
		const struct spi_mem_op *op)
{
	u32 total_tx_words, total_rx_words;
	int err = 0;

	if (op->data.buswidth == 4) {
		total_tx_words = 0;
		total_rx_words = (op->data.nbytes + 3) / 4;
		if (op->data.dir == SPI_MEM_DATA_OUT) {
			total_tx_words = (op->data.nbytes + 3) / 4;
			total_rx_words = 0;
		};

		scai_fpgaqspi_set_operate_mode(p, true);
		scai_fpgaqspi_start_transaction(p, total_tx_words, total_rx_words);

		if (op->data.dir == SPI_MEM_DATA_OUT) {
			p->tx_buf = (u8 *)op->data.buf.out;
			p->rx_buf = NULL;
			p->rx_len = 0;
			p->tx_len = op->data.nbytes;
			scai_fpgaqspi_write_op(p, true);
		} else if (op->data.dir == SPI_MEM_DATA_IN) {
			p->tx_buf = NULL;
			p->rx_buf = (u8 *)op->data.buf.in;
			p->rx_len = op->data.nbytes;
			p->tx_len = 0;
			scai_fpgaqspi_read_op(p, true);
		}
	}

	scai_fpgaqspi_finish_transaction(p, false);

	return err;
}

static int __do_exec_byte_op(struct scai_fpgaqspi_priv *p,
		const struct spi_mem_op *op)
{
	u32 address = op->addr.val;
	u8 opcode = op->cmd.opcode;
	u8 opaddr[32];
	u32 total_tx_bytes, total_rx_bytes;
	int err = 0, i;

	total_tx_bytes = op->cmd.nbytes + op->addr.nbytes + op->dummy.nbytes;
	total_rx_bytes = (op->data.buswidth == 1) ? op->data.nbytes : 0;
	if (op->data.dir == SPI_MEM_DATA_OUT && op->data.buswidth == 1) {
		total_tx_bytes += op->data.nbytes;
		total_rx_bytes -= op->data.nbytes;
	};

	scai_fpgaqspi_set_operate_mode(p, false);
	scai_fpgaqspi_start_transaction(p, total_tx_bytes, total_rx_bytes);

	if (op->cmd.opcode) {
		p->tx_buf = &opcode;
		p->rx_buf = NULL;
		p->tx_len = op->cmd.nbytes;
		p->rx_len = 0;
		scai_fpgaqspi_write_op(p, false);
	}

	if (op->addr.nbytes) {
		memset(opaddr, 0, sizeof(opaddr));
		p->tx_buf = &opaddr[0];
		for (i = 0; i < op->addr.nbytes; i++)
			p->tx_buf[i] = address >> (8 * (op->addr.nbytes - i - 1));

		p->rx_buf = NULL;
		p->tx_len = op->addr.nbytes;
		p->rx_len = 0;
		scai_fpgaqspi_write_op(p, false);
	}

	if (op->dummy.nbytes) {
		memset(opaddr, 0, sizeof(opaddr));
#if 0
		for (i = 0; i < op->dummy.nbytes; i++) {
			if (i > sizeof(opaddr)) {
				break;
			}
			opaddr[i] = 0;
		}
#endif

		p->tx_buf = &opaddr[0];
		p->rx_buf = NULL;
		p->tx_len = op->dummy.nbytes;
		p->rx_len = 0;
		scai_fpgaqspi_write_op(p, false);
	}

	if (op->data.nbytes && op->data.buswidth == 1) {
		if (op->data.dir == SPI_MEM_DATA_OUT) {
			p->tx_buf = (u8 *)op->data.buf.out;
			p->rx_buf = NULL;
			p->rx_len = 0;
			p->tx_len = op->data.nbytes;
			scai_fpgaqspi_write_op(p, false);
		} else if (op->data.dir == SPI_MEM_DATA_IN) {
			p->tx_buf = NULL;
			p->rx_buf = (u8 *)op->data.buf.in;
			p->rx_len = op->data.nbytes;
			p->tx_len = 0;
			scai_fpgaqspi_read_op(p, false);
		}
	}

	scai_fpgaqspi_finish_transaction(p, true);

	return err;
}

#if 0
static void dump_mem_op_info(const struct spi_mem_op *op)
{
	printk("\n");
	printk("==========================\n");
	printk("%s(%d):  op->cmd.opcode(0x%04X)\n",
			__func__, __LINE__, op->cmd.opcode);
	printk("%s(%d):  op->cmd.nbytes(0x%02X)\n",
			__func__, __LINE__, op->cmd.nbytes);
	printk("%s(%d):  op->cmd.buswidth(0x%02X)\n",
			__func__, __LINE__, op->cmd.buswidth);
	printk("%s(%d):  op->cmd.dtr(0x%02X)\n",
			__func__, __LINE__, op->cmd.dtr);
	printk("\n");

	printk("%s(%d):  op->addr.val(0x%016llX)\n",
			__func__, __LINE__, op->addr.val);
	printk("%s(%d):  op->addr.nbytes(0x%02X)\n",
			__func__, __LINE__, op->addr.nbytes);
	printk("%s(%d):  op->addr.buswidth(0x%02X)\n",
			__func__, __LINE__, op->addr.buswidth);
	printk("%s(%d):  op->addr.dtr(0x%02X)\n",
			__func__, __LINE__, op->addr.dtr);
	printk("\n");

	printk("%s(%d):  op->dummy.buswidth(0x%02X)\n",
			__func__, __LINE__, op->dummy.buswidth);
	printk("%s(%d):  op->dummy.dtr(0x%02X)\n",
			__func__, __LINE__, op->dummy.dtr);
	printk("%s(%d):  op->dummy.nbytes(0x%02X)\n",
			__func__, __LINE__, op->dummy.nbytes);
	printk("\n");

	printk("%s(%d):  op->data.buswidth(0x%02X)\n",
			__func__, __LINE__, op->data.buswidth);
	printk("%s(%d):  op->data.dtr(0x%02X)\n",
			__func__, __LINE__, op->data.dtr);
	printk("%s(%d):  op->data.nbytes(0x%08X)\n",
			__func__, __LINE__, op->data.nbytes);
	printk("==========================\n");
	printk("\n");
}
#endif

static int scai_fpgaqspi_exec_op(struct spi_mem *mem,
				 const struct spi_mem_op *op)
{
	struct scai_fpgaqspi_priv *p = spi_master_get_devdata(mem->spi->master);
	int err = 0;

	mutex_lock(&p->op_lock);
#if 0
	dump_mem_op_info(op);
#endif

	err = scai_fpgaqspi_wait_for_ready(p);
	if (err) {
		dev_err(&mem->spi->dev, "Timeout waiting on QSPI ready.\n");
		return err;
	}

	WARN_ON(op->addr.buswidth == 2 || op->data.buswidth == 2);

	err = __do_exec_byte_op(p, op);
	err = __do_exec_word_op(p, op);

	mutex_unlock(&p->op_lock);

	return err;
}

static int scai_fpgaqspi_adjust_op_size(struct spi_mem *mem,
		struct spi_mem_op *op)
{
	if (op->data.dir == SPI_MEM_DATA_OUT &&
			op->data.nbytes > MAX_DATA_CMD_LEN)
	{
		op->data.nbytes = MAX_DATA_CMD_LEN;
#if 0
		dev_info(&mem->spi->dev, "op->data.nbytes(0x%08X)\n", op->data.nbytes);
#endif
	}

	return 0;
}

static bool scai_fpgaqspi_supports_op(struct spi_mem *mem,
		const struct spi_mem_op *op)
{
	if (!spi_mem_default_supports_op(mem, op))
		return false;

	if ((op->data.buswidth == 2 || op->data.buswidth == 4) &&
	    (op->cmd.buswidth == 1 && (op->addr.buswidth <= 1)) &&
	    op->data.dir == SPI_MEM_DATA_OUT)
		return false;

	return true;
}

static const struct spi_controller_mem_ops scai_fpgaqspi_mem_ops = {
	.adjust_op_size = scai_fpgaqspi_adjust_op_size,
	.supports_op = scai_fpgaqspi_supports_op,
	.exec_op = scai_fpgaqspi_exec_op,
};

static int scai_fpgaqspi_setup(struct spi_device *spi)
{
	struct scai_fpgaqspi_priv *p = spi_master_get_devdata(spi->master);
	u32 control;

	control = CTRL1_RESET;
	p->ctrl1_sw_copy = control;
	writel(control, p->base_regs + SCAI_QSPI_REG_CTRL1);

	writel(0, p->base_regs + SCAI_QSPI_REG_CTRL2);
	writel(BIT(16), p->base_regs + SCAI_QSPI_REG_CTRL3);

	return 0;
}

static void scai_fpgaqspi_set_power(struct scai_fpgaqspi_priv *p,
		bool enable)
{
	u32 val1 = 0, val2 = 0;

	if (!p->gpio1_regs || !p->gpio2_regs) {
		return;
	}

	/* Read current GPIO state if needed
	 * val1 = readl(p->gpio1_regs + GPIO_REG_RDATA_OFFSET);
	 * val2 = readl(p->gpio2_regs + GPIO_REG_RDATA_OFFSET);
	 */
	if (enable) {
		val1 |= GPIO1_ENA_SS1_MASK;
		val2 |= GPIO2_ENA_SS2_MASK;
	} else {
		val1 &= ~GPIO1_ENA_SS1_MASK;
		val2 &= ~GPIO2_ENA_SS2_MASK;
	}

	writel(val1, p->gpio1_regs + GPIO_REG_WDATA_OFFSET);
	writel(val2, p->gpio2_regs + GPIO_REG_WDATA_OFFSET);
}

static int scai_fpgaqspi_probe(struct platform_device *pdev)
{
	struct spi_master *master = NULL;
	struct scai_fpgaqspi_priv *p = NULL;
	struct resource *r_qspi_base, *r_qspi_gpio1_base, *r_qspi_gpio2_base;
  resource_size_t size;
	u32 num_cs;
	int ret;

	master = spi_alloc_master(&pdev->dev, sizeof(*p));
	if (!master)
		return -ENOMEM;

	p = spi_controller_get_devdata(master);
	platform_set_drvdata(pdev, master);

	r_qspi_base = platform_get_resource_byname(pdev, IORESOURCE_MEM,
			"qspi_base");
	if (r_qspi_base == NULL) {
		r_qspi_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (r_qspi_base == NULL) {
			dev_err(&pdev->dev, "missing platform data\n");
			ret = -ENOMEM;
			goto out;
		}
	}
	size = (resource_size(r_qspi_base));
	dev_info(&pdev->dev, "r_qspi_base: start=%pa end=%pa size=%pa\n",
        &r_qspi_base->start, &r_qspi_base->end, &size);

	r_qspi_gpio1_base = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "qspi_gpio1_base");
	if (r_qspi_gpio1_base != NULL) {
		size = (resource_size(r_qspi_gpio1_base));
		dev_info(&pdev->dev, "r_qspi_gpio1_base: start=%pa end=%pa size=%pa\n",
				&r_qspi_gpio1_base->start, &r_qspi_gpio1_base->end,
				&size);
	}

	r_qspi_gpio2_base = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "qspi_gpio2_base");
	if (r_qspi_gpio2_base != NULL) {
		size = (resource_size(r_qspi_gpio2_base));
		dev_info(&pdev->dev, "r_qspi_gpio2_base: start=%pa end=%pa size=%pa\n",
				&r_qspi_gpio2_base->start, &r_qspi_gpio2_base->end,
				&size);
	}

	p->base_regs = devm_ioremap_resource(&pdev->dev, r_qspi_base);
	if (IS_ERR(p->base_regs)) {
		ret = dev_err_probe(&pdev->dev, PTR_ERR(p->base_regs),
				"failed to map registers\n");
		goto out;
	}
	dev_info(&pdev->dev, "qspi_base ioremap : 0x%px\n", p->base_regs);

	if (r_qspi_gpio1_base) {
		p->gpio1_regs = devm_ioremap_resource(&pdev->dev, r_qspi_gpio1_base);
		if (IS_ERR(p->gpio1_regs)) {
			ret = dev_err_probe(&pdev->dev, PTR_ERR(p->gpio1_regs),
					"failed to map registers\n");
			goto out;
		}
		dev_info(&pdev->dev, "qspi_gpio1_base ioremap : 0x%px(0x%08X)\n",
				p->gpio1_regs, readl(p->gpio1_regs + GPIO_REG_RDATA_OFFSET));
	}

	if (r_qspi_gpio2_base) {
		p->gpio2_regs = devm_ioremap_resource(&pdev->dev, r_qspi_gpio2_base);
		if (IS_ERR(p->gpio2_regs)) {
			ret = dev_err_probe(&pdev->dev, PTR_ERR(p->gpio2_regs),
					"failed to map registers\n");
			goto out;
		}
		dev_info(&pdev->dev, "qspi_gpio2_base ioremap : 0x%px(0x%08X)\n",
				p->gpio2_regs, readl(p->gpio2_regs + GPIO_REG_RDATA_OFFSET));
	}

	init_completion(&p->transfer_completion);
	mutex_init(&p->op_lock);

	master->bits_per_word_mask = SPI_BPW_MASK(8);
	master->mem_ops = &scai_fpgaqspi_mem_ops;
	master->setup = scai_fpgaqspi_setup;
	master->mode_bits = SPI_CPOL | SPI_CPHA | SPI_RX_QUAD | SPI_TX_QUAD;
	master->dev.of_node = pdev->dev.of_node;
	
	if (of_property_read_u32(pdev->dev.of_node, "num-cs", &num_cs))
		num_cs = 1;
		
	master->num_chipselect = num_cs;
	
	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret) {
		ret = dev_err_probe(&pdev->dev, ret,
				"spi_register_controller failed\n");
		goto out;
	}

	if (p->gpio1_regs && p->gpio2_regs) {
		scai_fpgaqspi_set_power(p, true);
	}

	return 0;

out:
	spi_master_put(master);
	return ret;
}

static void scai_fpgaqspi_remove(struct platform_device *pdev)
{
	struct scai_fpgaqspi_priv *p = platform_get_drvdata(pdev);

	if (p->gpio1_regs && p->gpio2_regs)
		scai_fpgaqspi_set_power(p, false);
}

#define SCAI_FPGAQSPI_PM_OPS (NULL)

#if defined(CONFIG_OF)
static const struct of_device_id scai_fpgaqspi_of_match[] = {
	{ .compatible = "scai-fpgaqspi,navc-mt29f" },
	{ .compatible = "scai-fpgaqspi,navc-backup-w25" },
	{ .compatible = "scai-fpgaqspi,navc-nor" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, scai_fpgaqspi_of_match);
#endif

static struct platform_driver scai_fpgaqspi_driver = {
	.probe     = scai_fpgaqspi_probe,
	.driver = {
		.name	= "scai-fpgaqspi",
		.pm = SCAI_FPGAQSPI_PM_OPS,
		.of_match_table = of_match_ptr(scai_fpgaqspi_of_match),
	},
	.remove_new = scai_fpgaqspi_remove,
};

module_platform_driver(scai_fpgaqspi_driver);
MODULE_AUTHOR("Sourav Poddar <sourav.poddar@ti.com>");
MODULE_DESCRIPTION("TI QSPI controller driver");
MODULE_LICENSE("GPL");
