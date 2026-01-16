#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/physmap.h>
#include <linux/mtd/concat.h>
#include <linux/mtd/cfi_endian.h>
#include <linux/io.h>
#include <linux/of_device.h>

#include "../../base/base.h"

struct mtd_concat_scai {
	int num_subdev;
	struct mtd_info **mtds;
	struct mtd_info *cmtd;
	const char *dev_name;
};

static int mtd_concat_scai_remove(struct platform_device *pdev)
{
	struct mtd_concat_scai *drvdata = NULL;
	int i = 0;

	drvdata = platform_get_drvdata(pdev);

	if (drvdata->cmtd) {
		sysfs_remove_link(&drvdata->cmtd->dev.kobj, drvdata->cmtd->name);
		WARN_ON(mtd_device_unregister(drvdata->cmtd));

		if (drvdata->cmtd != drvdata->mtds[0])
			mtd_concat_destroy(drvdata->cmtd);
	}

	for (i = 0; i < drvdata->num_subdev; i++) {
		if (drvdata->mtds[i])
			map_destroy(drvdata->mtds[i]);
	}

	return 0;
}

static int mtd_concat_scai_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mtd_concat_scai *drvdata = NULL;
	int err = 0;
	int i = 0;

	if (!pdev->dev.of_node && !dev_get_platdata(&pdev->dev)) {
		err = -EINVAL;
		goto out;
	}

	drvdata = devm_kzalloc(&pdev->dev, sizeof(struct mtd_concat_scai),
			GFP_KERNEL);
	if (!drvdata) {
		dev_err(&pdev->dev, "failed to alloc(struct mtd_concat_scai)...\n");
		err = -ENOMEM;
		goto out;
	}

	if (of_property_read_u32(np, "num-subdev", &drvdata->num_subdev)) {
		dev_err(&pdev->dev, "failed to get num_subdev(%d) ...\n",
				drvdata->num_subdev);
		drvdata->num_subdev = 0;
		err = -EINVAL;
		goto out;
	}

	if (drvdata->num_subdev <= 0)
		goto out;

	drvdata->mtds = devm_kzalloc(&pdev->dev,
			sizeof(struct mtd_info) * drvdata->num_subdev,
			GFP_KERNEL);
	if (!drvdata->mtds) {
		dev_err(&pdev->dev, "failed to alloc(struct mtd_info)...\n");
		err = -ENOMEM;
		goto out;
	}

	drvdata->dev_name = of_get_property(np, "device-name", NULL);
	if (!drvdata->dev_name)
		drvdata->dev_name = "scai_mtd_default";

	platform_set_drvdata(pdev, drvdata);

	for (i = 0; i < drvdata->num_subdev; i++) {
		struct device_node *subnp;

		subnp = of_parse_phandle(np, "subdevices", i);
		if (!subnp) {
			dev_err(&pdev->dev, "failed to get subdevices list...\n");
			err = -EINVAL;
			goto out;
		}

		drvdata->mtds[i] = of_get_mtd_device_by_node(subnp);
		if (IS_ERR(drvdata->mtds[i])) {
			dev_err(&pdev->dev, "failed to get_mtd_device(subnp), wait probe().\n");
			of_node_put(subnp);
			err = -EPROBE_DEFER;
			goto out;
		}

		of_node_put(subnp);
	}

	drvdata->cmtd = mtd_concat_create(drvdata->mtds, drvdata->num_subdev,
			drvdata->dev_name);
	if (!drvdata->cmtd) {
		dev_err(&pdev->dev, "failed to do mtd_concat_create()...\n");
		err = -ENXIO;
		goto out;
	}

	mtd_device_register(drvdata->cmtd, NULL, 0);

	struct subsys_private *sp;
	sp = class_to_subsys(drvdata->cmtd->dev.class);

	sysfs_create_link(&sp->subsys.kobj,
			&drvdata->cmtd->dev.kobj, drvdata->cmtd->name);

out:
	return err;
}

static const struct of_device_id of_flash_match[] = {
	{
		.compatible = "mtd-concat-scai",
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, of_flash_match);


static struct platform_driver mtd_concat_scai_driver = {
	.probe		= mtd_concat_scai_probe,
	.remove		= mtd_concat_scai_remove,
	.driver		= {
		.name	= "mtd-concat-scai",
		.of_match_table = of_flash_match,
	},
};

static int __init mtd_concat_scai_init(void)
{
	int err;

	err = platform_driver_register(&mtd_concat_scai_driver);

	return err;
}

static void __exit mtd_concat_scai_exit(void)
{
	platform_driver_unregister(&mtd_concat_scai_driver);
}

module_init(mtd_concat_scai_init);
module_exit(mtd_concat_scai_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kwangsu Jung <kwangsu.jung@tii.ae>");
MODULE_AUTHOR("Kwangsu Jung <kwangsu.jung@unikie.com>");
MODULE_DESCRIPTION("SCAI MTD concat driver");

