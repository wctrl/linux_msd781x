// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Daniel Palmer
 */

#include <linux/clk-provider.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include "clk-msc313-mux.h"

static const struct msc313_mux_data msc313_muxes[] = {
};

static const struct msc313_muxes_data msc313_data = MSC313_MUXES_DATA(msc313_muxes);

static const struct msc313_mux_data ssd20xd_muxes[] = {
};

static const struct msc313_muxes_data ssd20xd_data = MSC313_MUXES_DATA(ssd20xd_muxes);

static const struct of_device_id msc313_pm_muxes_ids[] = {
	{
		.compatible = "mstar,msc313-pm-muxes",
		.data = &msc313_data,
	},
	{
		.compatible = "sstar,ssd20xd-pm-muxes",
		.data = &ssd20xd_data,
	},
	{}
};

static int msc313_pm_muxes_probe(struct platform_device *pdev)
{
	const struct msc313_muxes_data *match_data;
	struct device *dev = &pdev->dev;
	struct regmap *regmap;

	match_data = of_device_get_match_data(dev);
	if (!match_data)
		return -EINVAL;

	regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return msc313_mux_register_muxes(dev, regmap, match_data);
}

static struct platform_driver msc313_pm_muxes_driver = {
	.driver = {
		.name = "msc313-pm-muxes",
		.of_match_table = msc313_pm_muxes_ids,
	},
	.probe = msc313_pm_muxes_probe,
};
builtin_platform_driver(msc313_pm_muxes_driver);
