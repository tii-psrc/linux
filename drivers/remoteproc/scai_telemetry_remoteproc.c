// SPDX-License-Identifier: GPL-2.0
/*
 * scai_telemetry_remoteproc.c
 *
 * sbi_ecall driver for telemetry rproc
 *
 */


#include <asm/sbi.h>
#include <asm/vendorid_list.h>

#include <linux/dma-mapping.h>

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>


#define SBI_EXT_MICROCHIP_TECHNOLOGY	(SBI_EXT_VENDOR_START | \
		MICROCHIP_VENDOR_ID)

enum {
	SBI_EXT_TELEMETRY_RPROC_COMMAND = 0x14
};

enum sbi_tm_ext_cmd {
	SBI_TM_EXT_CONCISE = 0x0,
	SBI_TM_EXT_VERBOSE = 0x1,
	SBI_TM_EXT_STOP_PUBLISHING = 0x2,
};

struct scai_tm_rproc_drv {
	struct device *dev;
	struct miscdevice misc;
	void *cpu_addr;
	phys_addr_t phys_addr;
	size_t size;
};

struct user_data {
	unsigned long arg0;
	long size;
	void *buf;
};


static long scai_tm_rproc_misc_ioctl(struct file *file,
                                     unsigned int cmd,
                                     unsigned long arg)
{
	struct scai_tm_rproc_drv *drv = file->private_data;
	struct user_data user_data;
	struct sbiret ret;
	long copy_size;
	int err;

	pr_info("%s: cmd=0x%x\n", __func__, cmd);

	memset_io(drv->cpu_addr, 0, drv->size);

	switch (cmd) {
	case SBI_EXT_TELEMETRY_RPROC_COMMAND:

		if (copy_from_user(&user_data,
				   (void __user *)arg,
				   sizeof(user_data)))
			return -EFAULT;
		pr_info("%s: arg0=0x%x\n", __func__, user_data.arg0);

		ret = sbi_ecall(SBI_EXT_MICROCHIP_TECHNOLOGY,
				SBI_EXT_TELEMETRY_RPROC_COMMAND,
				user_data.arg0,
				(unsigned long)drv->phys_addr, 0, 0, 0, 0);
		break;

	default:
		return -EINVAL;
	}

	if (ret.error)
		return sbi_err_map_linux_errno(ret.error);

	if (user_data.arg0 == SBI_TM_EXT_STOP_PUBLISHING)
		return 0;

	copy_size = min_t(long, ret.value, user_data.size);

	pr_info("buffer=%p user=%p size=%ld actual=%ld\n",
		drv->cpu_addr,
		user_data.buf,
		user_data.size,
		copy_size);
	print_hex_dump(KERN_INFO,
			"HSS BUF: ",
			DUMP_PREFIX_OFFSET,
			16,
			1,
			drv->cpu_addr,
			copy_size,
			true);

	err = copy_to_user(user_data.buf,
			   drv->cpu_addr,
			   copy_size);
	if (err) {
		pr_err("copy_to_user(buffer) failed (%d bytes not copied)\n",
		       err);
		return -EFAULT;
	}

	/* Return actual copied size */
	user_data.size = copy_size;

	err = copy_to_user((void __user *)arg,
			   &user_data,
			   sizeof(user_data));
	if (err) {
		pr_err("copy_to_user(user_data) failed (%d bytes not copied)\n",
		       err);
		return -EFAULT;
	}

	return copy_size;
}

static int scai_tm_rproc_misc_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct scai_tm_rproc_drv *drv = container_of(misc, struct scai_tm_rproc_drv, misc);

	file->private_data = drv;
	return 0;
}

static const struct file_operations scai_tm_rproc_misc_fops = {
	.owner		= THIS_MODULE,
	.open     = scai_tm_rproc_misc_open,
	.unlocked_ioctl	= scai_tm_rproc_misc_ioctl,
};

static int scai_tm_rproc_probe(struct platform_device *pdev)
{
	struct resource res;
	struct device_node *mem_np;
	struct scai_tm_rproc_drv *drv;
	int ret;

	drv = devm_kzalloc(&pdev->dev,
			sizeof(*drv),
			GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	drv->dev = &pdev->dev;

	mem_np = of_parse_phandle(pdev->dev.of_node,
			"memory-region",
			0);
	if (!mem_np) {
		dev_err(&pdev->dev,
				"missing memory-region\n");
		return -ENODEV;
	}

	ret = of_address_to_resource(mem_np, 0, &res);
	of_node_put(mem_np);

	if (ret) {
		dev_err(&pdev->dev,
				"failed to get reserved memory resource\n");
		return ret;
	}

	drv->phys_addr = res.start;
	drv->size = resource_size(&res);

	dev_info(&pdev->dev,
			"HSS buffer PA=%pa size=%zu\n",
			&drv->phys_addr,
			drv->size);


	drv->cpu_addr = devm_ioremap(&pdev->dev,
			drv->phys_addr,
			drv->size);

	if (!drv->cpu_addr) {
		dev_err(&pdev->dev,
				"failed to map shared memory\n");
		return -ENOMEM;
	}


	drv->misc.minor = MISC_DYNAMIC_MINOR;
	drv->misc.name = "scai_tm_rproc";
	drv->misc.fops = &scai_tm_rproc_misc_fops;
	drv->misc.mode = 0666;

	ret = misc_register(&drv->misc);
	if (ret) {
		dev_err(&pdev->dev,
				"misc_register failed %d\n",
				ret);
		return ret;
	}


	platform_set_drvdata(pdev, drv);

	return 0;
}

static void scai_tm_rproc_remove(struct platform_device *pdev)
{
	struct scai_tm_rproc_drv *p = platform_get_drvdata(pdev);

	misc_deregister(&p->misc);

	dev_info(&pdev->dev, "scai_rproc removed\n");
}

#define SCAI_TM_RPROC_PM_OPS (NULL)

#if defined(CONFIG_OF)
static const struct of_device_id scai_tm_remoteproc_of_match[] = {
	{ .compatible = "tii,scai-tm-rproc"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, scai_tm_remoteproc_of_match);
#endif

static struct platform_driver scai_rproc_driver = {
	.probe     = scai_tm_rproc_probe,
	.driver = {
		.name	= "scai-tm-rproc",
		.pm = SCAI_TM_RPROC_PM_OPS,
		.of_match_table = of_match_ptr(scai_tm_remoteproc_of_match),
	},
	.remove_new = scai_tm_rproc_remove,
};

module_platform_driver(scai_rproc_driver);
MODULE_AUTHOR("Kwangsu Jung <kwangsu.jung@tii.ae>");
MODULE_AUTHOR("Kwangsu Jung <kwangsu.jung@unikie.com>");
MODULE_DESCRIPTION("Microchip PolarFire SoC SCAI RPROC driver");
MODULE_LICENSE("GPL");
