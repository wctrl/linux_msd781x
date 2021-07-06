#include <drm/drm_crtc.h>
#include <linux/component.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define DRIVER_NAME "mstar-op2"

struct mstar_op2 {
	struct drm_crtc *crtc;
};

static const struct regmap_config mstar_op2_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static int mstar_op2_bind(struct device *dev, struct device *master,
			 void *data)
{
	struct drm_device *drm = data;

	printk("bind op2\n");
	return 0;
}

static void mstar_op2_unbind(struct device *dev, struct device *master,
			    void *data)
{
}

static const struct component_ops mstar_op2_component_ops = {
	.bind	= mstar_op2_bind,
	.unbind	= mstar_op2_unbind,
};

static int mstar_op2_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mstar_op2 *op2;
	struct regmap *regmap;
	void __iomem *base;

	dev_info(dev, "probe\n");

	op2 = devm_kzalloc(dev, sizeof(*op2), GFP_KERNEL);
	if (!op2)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(dev, base, &mstar_op2_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return component_add(&pdev->dev, &mstar_op2_component_ops);
}

static int mstar_op2_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mstar_op2_component_ops);
	return 0;
}

static const struct of_device_id mstar_op2_ids[] = {
	{
		.compatible = "sstar,ssd20xd-op2",
	},
	{},
};
MODULE_DEVICE_TABLE(of, mstar_op2_ids);

static struct platform_driver mstar_op2_driver = {
	.probe = mstar_op2_probe,
	.remove = mstar_op2_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = mstar_op2_ids,
	},
};
module_platform_driver(mstar_op2_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
