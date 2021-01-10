// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2020 Daniel Palmer
 */

#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <soc/mstar/pmsleep.h>
#include <dt-bindings/pinctrl/mstar.h>

#include "../core.h"
#include "../devicetree.h"
#include "../pinconf.h"
#include "../pinmux.h"

#include "pinctrl-mstar.h"

#define DRIVER_NAME "pinctrl-mstar-pm"

struct msc313_pinctrl {
	struct device *dev;
	struct pinctrl_desc desc;
	struct pinctrl_dev *pctl;
	struct regmap *regmap;
	const struct msc313_pinctrl_info *info;
};

struct msc313_pinctrl_function {
	const char *name;
	int reg;
	u16 mask;
	const char * const *groups;
	const u16 *values;
	int numgroups;
};

#define MSTAR_PINCTRL_FUNCTION(n, r, m, g, v) \
	{ \
		.name = n,\
		.reg = r,\
		.mask = m,\
		.groups = g,\
		.values = v,\
		.numgroups = ARRAY_SIZE(g)\
	}

struct msc313_pinctrl_group {
	const char *name;
	const int *pins;
	const int numpins;
};

#define MSTAR_PINCTRL_GROUP(n, p) \
	{\
		.name = n,\
		.pins = p,\
		.numpins = ARRAY_SIZE(p)\
	}

/*
 * Not all pins have "pinconf" so we only
 * carry this extra info for those that do.
 *
 * For some pins all of these bits in the same
 * register, for others they are split across
 * registers. Not all pins have all of the
 * registers.
 */
struct msc313_pinctrl_pinconf {
	const int pin;
	const int pull_en_reg;
	const int pull_en_bit;
	const int pull_dir_reg;
	const int pull_dir_bit;
	const int input_reg;
	const int input_bit;
	const int drive_reg;
	const int drive_lsb;
	const int drive_width;
	const unsigned int *drivecurrents;
	const int ndrivecurrents;
};

/*
 * Per-chip info that describes all of the pins,
 * the pin groups, the mappable functions and
 * pins that support pinconf.
 */
struct msc313_pinctrl_info {
	const struct pinctrl_pin_desc *pins;
	const int npins;
	const struct msc313_pinctrl_group *groups;
	const int ngroups;
	const struct msc313_pinctrl_function *functions;
	const int nfunctions;
	const struct msc313_pinctrl_pinconf *pinconfs;
	const int npinconfs;
};

#define MSTAR_PINCTRL_INFO(_chip) static const struct msc313_pinctrl_info _chip##_info = { \
	.pins = _chip##_pins, \
	.npins = ARRAY_SIZE(_chip##_pins), \
	.groups = _chip##_pinctrl_groups, \
	.ngroups = ARRAY_SIZE(_chip##_pinctrl_groups), \
	.functions = _chip##_pinctrl_functions, \
	.nfunctions = ARRAY_SIZE(_chip##_pinctrl_functions), \
	.pinconfs = _chip##_configurable_pins, \
	.npinconfs = ARRAY_SIZE(_chip##_configurable_pins),\
}

#if CONFIG_MACH_INFINITY
#define MSC313_COMMON_PIN(_pinname) COMMON_PIN(MSC313, _pinname)
/* msc313/msc313e */

/* pinctrl pins */
static struct pinctrl_pin_desc msc313_pins[] = {
};

/* mux pin groupings */


#define MSC313_PINCTRL_GROUP(_NAME, _name) MSTAR_PINCTRL_GROUP(GROUPNAME_##_NAME, msc313_##_name##_pins)

static const struct msc313_pinctrl_group msc313_pinctrl_groups[] = {
};

static const struct msc313_pinctrl_function msc313_pinctrl_functions[] = {
};

static const struct msc313_pinctrl_pinconf msc313_configurable_pins[] = {
};

MSTAR_PINCTRL_INFO(msc313);

/* ssd20xd */

/* pinctrl pins */
static const struct pinctrl_pin_desc ssd20xd_pins[] = {
	SSD20XD_COMMON_PIN(PM_LED0),
	SSD20XD_COMMON_PIN(PM_LED1),
};

/* mux pin groupings */
static const int ssd20xd_pm_led_mode1_pins[] = {
	PIN_SSD20XD_PM_LED0,
	PIN_SSD20XD_PM_LED1,
};

static const struct msc313_pinctrl_group ssd20xd_pinctrl_groups[] = {
	SSD20XD_PINCTRL_GROUP(PM_LED_MODE1, pm_led_mode1)
};

/* chip specific functions */
#define REG_SSD20XD_PM_LED MSTAR_PMSLEEP_PMLED
#define SHIFT_SSD20XD_PM_LED 4
#define WIDTH_SSD20XD_PM_LED 2
#define MASK_SSD20XD_PM_LED MAKEMASK(SSD20XD_PM_LED)

static const char * const ssd20xd_pm_led_groups[] = {
	GROUPNAME_PM_LED_MODE1,
};

static const u16 ssd20xd_pm_led_values[] = {
	SSD20XD_MODE(PM_LED, 1),
};

static const struct msc313_pinctrl_function ssd20xd_pinctrl_functions[] = {
	SSD20XD_FUNCTION(PM_LED, pm_led),
};

static const struct msc313_pinctrl_pinconf ssd20xd_configurable_pins[] = {
};

MSTAR_PINCTRL_INFO(ssd20xd);
#endif /* infinity */

#ifdef CONFIG_MACH_MERCURY
/* ssc8336 */
#define SSC8336N_COMMON_PIN(_pinname) COMMON_PIN(SSC8336N, _pinname)

/* pinctrl pins */
static const struct pinctrl_pin_desc ssc8336n_pins[] = {
};

/* mux pin groupings */

#define SSC8336N_PINCTRL_GROUP(_NAME, _name) MSTAR_PINCTRL_GROUP(GROUPNAME_##_NAME, ssc8336n_##_name##_pins)

/* pinctrl groups */

static const struct msc313_pinctrl_group ssc8336n_pinctrl_groups[] = {
};

#define SSC8336N_FUNCTION(_NAME, _name) \
	MSTAR_PINCTRL_FUNCTION(FUNCTIONNAME_##_NAME, REG_SSC8336N_##_NAME, \
	MASK_SSC8336N_##_NAME, ssc8336n_##_name##_groups, ssc8336n_##_name##_values)

static const struct msc313_pinctrl_function ssc8336n_pinctrl_functions[] = {
};

static const struct msc313_pinctrl_pinconf ssc8336n_configurable_pins[] = {
};

MSTAR_PINCTRL_INFO(ssc8336n);
#endif /* mercury5 */

static int mstar_dt_node_to_map(struct pinctrl_dev *pctldev,
			       struct device_node *np,
			       struct pinctrl_map **map,
			       unsigned int *num_maps)
{
	return pinconf_generic_dt_node_to_map(pctldev, np,
						map, num_maps,
						PIN_MAP_TYPE_INVALID);
}

static void mstar_dt_free_map(struct pinctrl_dev *pctldev,
			     struct pinctrl_map *map, unsigned int num_maps)
{
	kfree(map);
}

static const struct pinctrl_ops msc313_pinctrl_ops = {
	.get_groups_count	= pinctrl_generic_get_group_count,
	.get_group_name		= pinctrl_generic_get_group_name,
	.get_group_pins		= pinctrl_generic_get_group_pins,
	.dt_node_to_map		= mstar_dt_node_to_map,
	.dt_free_map		= mstar_dt_free_map,
};

static int mstar_set_mux(struct pinctrl_dev *pctldev, unsigned int func,
			   unsigned int group)
{
	struct msc313_pinctrl *pinctrl = pctldev->driver_data;
	const char *grpname = pinctrl_generic_get_group_name(pctldev, group);
	struct function_desc *funcdesc = pinmux_generic_get_function(pctldev, func);
	struct msc313_pinctrl_function *function = funcdesc->data;
	int i, ret = 0;

	if(function != NULL){
		if(function->reg >= 0 && function->values != NULL){
			for(i = 0; i < function->numgroups; i++){
				if(strcmp(function->groups[i], grpname) == 0){
					dev_dbg(pinctrl->dev, "updating mux reg %x\n", (unsigned) function->reg);
					ret = regmap_update_bits(pinctrl->regmap, function->reg,
							function->mask, function->values[i]);
					if(ret)
						dev_dbg(pinctrl->dev, "failed to update register\n");
					break;
				}
			}
		}
		else {
			dev_dbg(pinctrl->dev, "reg or values not found\n");
		}
	}
	else {
		dev_info(pinctrl->dev, "missing function data\n");
	}

	return ret;
}

static const struct pinmux_ops mstar_pinmux_ops = {
	.get_functions_count = pinmux_generic_get_function_count,
	.get_function_name   = pinmux_generic_get_function_name,
	.get_function_groups = pinmux_generic_get_function_groups,
	.set_mux             = mstar_set_mux,
	.strict              = true,
};

static int msc313_pinctrl_parse_groups(struct msc313_pinctrl *pinctrl){
	int i, ret;

	for(i = 0; i < pinctrl->info->ngroups; i++){
		const struct msc313_pinctrl_group *grp = &pinctrl->info->groups[i];
		ret = pinctrl_generic_add_group(pinctrl->pctl, grp->name,
				(int*) grp->pins, grp->numpins, NULL);
	}
	return ret;
}

static int msc313_pinctrl_parse_functions(struct msc313_pinctrl *pinctrl){
	int i, ret;

	for(i = 0; i < pinctrl->info->nfunctions; i++){
		const struct msc313_pinctrl_function *func =  &pinctrl->info->functions[i];

		// clear any existing value for the function
		if(func->reg >= 0){
			regmap_update_bits(pinctrl->regmap, func->reg,
					func->mask, 0);
		}

		ret = pinmux_generic_add_function(pinctrl->pctl, func->name,
				(const char**) func->groups, func->numgroups, (void*) func);
		if(ret < 0){
			dev_err(pinctrl->dev, "failed to add function: %d", ret);
			goto out;
		}
	}
	out:
	return ret;
}

static const struct regmap_config msc313_pinctrl_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static int msc313_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct msc313_pinctrl *pinctrl;
	const struct msc313_pinctrl_info *match_data;
	int ret;

	match_data = of_device_get_match_data(&pdev->dev);
	if (!match_data)
		return -EINVAL;

	pinctrl = devm_kzalloc(&pdev->dev, sizeof(*pinctrl), GFP_KERNEL);
	if (!pinctrl)
		return -ENOMEM;

	platform_set_drvdata(pdev, pinctrl);

	pinctrl->dev = &pdev->dev;
	pinctrl->info = match_data;

	pinctrl->regmap = syscon_regmap_lookup_by_phandle(dev->of_node, "mstar,pmsleep");
	if (IS_ERR(pinctrl->regmap)) {
		dev_err(pinctrl->dev, "failed to register regmap");
		return PTR_ERR(pinctrl->regmap);
	}

	pinctrl->desc.name = DRIVER_NAME;
	pinctrl->desc.pctlops = &msc313_pinctrl_ops;
	pinctrl->desc.pmxops = &mstar_pinmux_ops;
	pinctrl->desc.owner = THIS_MODULE;
	pinctrl->desc.pins = pinctrl->info->pins;
	pinctrl->desc.npins = pinctrl->info->npins;

	ret = devm_pinctrl_register_and_init(pinctrl->dev, &pinctrl->desc,
					     pinctrl, &pinctrl->pctl);

	if (ret) {
		dev_err(pinctrl->dev, "failed to register pinctrl\n");
		return ret;
	}

	ret = msc313_pinctrl_parse_functions(pinctrl);
	ret = msc313_pinctrl_parse_groups(pinctrl);

	ret = pinctrl_enable(pinctrl->pctl);
	if (ret)
		dev_err(pinctrl->dev, "failed to enable pinctrl\n");

	return 0;
}

static const struct of_device_id msc313_pinctrl_of_match[] = {
#if CONFIG_MACH_INFINITY
	{
		.compatible	= "mstar,msc313-pm-pinctrl",
		.data		= &msc313_info,
	},
	{
		.compatible	= "mstar,msc313e-pm-pinctrl",
		.data		= &msc313_info,
	},
	{
		.compatible	= "sstar,ssd20xd-pm-pinctrl",
		.data		= &ssd20xd_info,
	},
#endif
#ifdef CONFIG_MACH_MERCURY
	{
		.compatible	= "mstar,ssc8336-pm-pinctrl",
		.data		= &ssc8336n_info,
	},
	{
		.compatible	= "mstar,ssc8336n-pm-pinctrl",
		.data		= &ssc8336n_info,
	},
#endif
	{ }
};
MODULE_DEVICE_TABLE(of, msc313_pinctrl_of_match);

static struct platform_driver msc313_pinctrl_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = msc313_pinctrl_of_match,
	},
	.probe = msc313_pinctrl_probe,
};
module_platform_driver(msc313_pinctrl_driver);

MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
MODULE_DESCRIPTION("PM Pin controller driver for MStar SoCs");
MODULE_LICENSE("GPL v2");
