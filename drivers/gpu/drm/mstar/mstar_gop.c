/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_plane.h>
#include <linux/component.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>

#include "mstar_gop.h"

#define DRIVER_NAME "mstar-gop"

struct mstar_gop_data {
	const uint32_t *formats;
	const unsigned int num_formats;
	const enum drm_plane_type type;
};

struct mstar_gop {
	struct device *dev;
	struct regmap *regmap;
	struct clk *fclk; /* vendor code says this is only needed when setting the palette */
	struct regmap_field *dst;
	struct drm_plane drm_plane;
	const struct mstar_gop_data *data;
};

static const struct regmap_config mstar_gop_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static irqreturn_t mstar_gop_irq(int irq, void *data)
{
	return IRQ_HANDLED;
}

static const char* dsts[] = {
		"ip_main", "ip_sub", "op", "mvop", "sub_mvop", "unknown", "frc", "unknown"
};

static void mstar_gop_dump(struct mstar_gop *gop){
	unsigned int val;
	regmap_field_read(gop->dst, &val);
	dev_info(gop->dev, "dst: %s", dsts[val]);
}


static void mstar_gop_reset(struct mstar_gop *gop){
	dev_info(gop->dev, "reset");
	regmap_update_bits(gop->regmap, MSTAR_GOP_REG_CONFIG,
			MSTAR_GOP_REG_CONFIG_RST, MSTAR_GOP_REG_CONFIG_RST);
	mdelay(10);
	regmap_update_bits(gop->regmap, MSTAR_GOP_REG_CONFIG,
			MSTAR_GOP_REG_CONFIG_RST, 0);

	mstar_gop_dump(gop);
}

static int gop_plane_atomic_check(struct drm_plane *plane,
				    struct drm_atomic_state *state)
{
	printk("%s\n", __func__);
	return 0;
}

static void gop_plane_atomic_update(struct drm_plane *plane,
				      struct drm_atomic_state *state)
{
	printk("%s\n", __func__);
}

static const struct drm_plane_helper_funcs gop_plane_helper_funcs = {
	.atomic_check = gop_plane_atomic_check,
	.atomic_update = gop_plane_atomic_update,
};

static const struct drm_plane_funcs gop_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.destroy		= drm_plane_cleanup,
	.reset			= drm_atomic_helper_plane_reset,
	.atomic_duplicate_state = drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

static const uint64_t mop_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static int mstar_gop_bind(struct device *dev, struct device *master,
			 void *data)
{
	struct mstar_gop *gop = dev_get_drvdata(dev);
	struct drm_device *drm_device = data;
	int ret;

	ret = drm_universal_plane_init(drm_device,
				     &gop->drm_plane,
				     0,
				     &gop_plane_funcs,
				     gop->data->formats,
				     gop->data->num_formats,
				     mop_format_modifiers,
				     gop->data->type,
				     NULL);
	if(ret)
		return ret;

	drm_plane_helper_add(&gop->drm_plane, &gop_plane_helper_funcs);

	return 0;
}

static void mstar_gop_unbind(struct device *dev, struct device *master,
			    void *data)
{
}

static const struct component_ops mstar_gop_ops = {
	.bind	= mstar_gop_bind,
	.unbind	= mstar_gop_unbind,
};

static int mstar_gop_probe(struct platform_device *pdev)
{
	const struct mstar_gop_data *match_data;
	struct device *dev = &pdev->dev;
	struct mstar_gop *gop;
	__iomem void *regs;
	int irq, ret;

	match_data = of_device_get_match_data(dev);
	if (!match_data)
		return -EINVAL;

	gop = devm_kzalloc(dev, sizeof(*gop), GFP_KERNEL);
	if(!gop){
		return -ENOMEM;
	}

	gop->data = match_data;
	gop->dev = dev;

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	gop->regmap = devm_regmap_init_mmio(dev, regs, &mstar_gop_regmap_config);
	if(IS_ERR(gop->regmap))
		return PTR_ERR(gop->regmap);

	gop->dst = regmap_field_alloc(gop->regmap, gop_dst_field);

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!irq){
		dev_info(gop->dev, "no interrupt provided");
		goto no_irq;
	}
	else {
		ret = devm_request_irq(&pdev->dev, irq, mstar_gop_irq, IRQF_SHARED,
			dev_name(&pdev->dev), gop);
		if (ret)
			return ret;
	}

no_irq:
	gop->fclk = of_clk_get(pdev->dev.of_node, 0);
	if (IS_ERR(gop->fclk)) {
		//return PTR_ERR(gop->fclk);
	}

	dev_set_drvdata(dev, gop);

	mstar_gop_reset(gop);

	return component_add(&pdev->dev, &mstar_gop_ops);
}

static int mstar_gop_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mstar_gop_ops);
	return 0;
}

static const uint32_t ssd20xd_gop0_formats[] = {
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_ARGB1555,
};

static const uint32_t ssd20xd_gop1_formats[] = {
	DRM_FORMAT_YUV422,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_ARGB4444,
	DRM_FORMAT_ARGB1555,
};

static const struct mstar_gop_data ssd20xd_gop0_data = {
	.formats = ssd20xd_gop0_formats,
	.num_formats = ARRAY_SIZE(ssd20xd_gop0_formats),
	.type = DRM_PLANE_TYPE_CURSOR,
};

static const struct mstar_gop_data ssd20xd_gop1_data = {
	.formats = ssd20xd_gop1_formats,
	.num_formats = ARRAY_SIZE(ssd20xd_gop1_formats),
	.type = DRM_PLANE_TYPE_PRIMARY,
};

static const struct of_device_id mstar_gop_dt_ids[] = {
	{
		.compatible = "sstar,ssd20xd-gop0",
		.data = &ssd20xd_gop0_data,
	},
	{
		.compatible = "sstar,ssd20xd-gop1",
		.data = &ssd20xd_gop1_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, mstar_gop_dt_ids);

static struct platform_driver mstar_gop_driver = {
	.probe = mstar_gop_probe,
	.remove = mstar_gop_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = mstar_gop_dt_ids,
	},
};

module_platform_driver(mstar_gop_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
