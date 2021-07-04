#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define DRIVER_NAME "mstar-mop"

#define ADDR_SHIFT	4

#define WINDOWS_NUM	16
#define	WINDOWS_START	0x200
#define	WINDOW_LEN	0x40


struct mstar_mop_window {
	struct regmap_field *en;
	struct regmap_field *yaddrl, *yaddrh;
	struct regmap_field *caddrl, *caddrh;
	struct regmap_field *hst;
	struct regmap_field *hend;
	struct regmap_field *vst;
	struct regmap_field *vend;
	struct regmap_field *pitch;
	struct regmap_field *src_width;
	struct regmap_field *src_height;
	struct regmap_field *scale_h;
	struct regmap_field *scale_v;
};

struct mstar_mop {
	struct regmap_field *swrst;
	struct regmap_field *gw_hsize;
	struct regmap_field *gw_vsize;
	struct mstar_mop_window windows[WINDOWS_NUM];
};

struct reg_field swrst_field = REG_FIELD(0x0, 0, 0);
struct reg_field gw_hsize_field = REG_FIELD(0x1c, 0, 12);
struct reg_field gw_vsize_field = REG_FIELD(0x20, 0, 12);

static const struct regmap_config mstar_mop_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4,
};

static void mstar_mop_dump_window(struct device *dev, struct mstar_mop_window *win)
{
	unsigned int en;
	unsigned int yaddrl, yaddrh;
	unsigned int caddrl, caddrh;
	unsigned int hst, hend;
	unsigned int vst, vend;
	unsigned int pitch;
	unsigned int srcw, srch;
	unsigned int scaleh, scalev;
	u32 yaddr, caddr;

	regmap_field_read(win->en, &en);

	regmap_field_read(win->yaddrl, &yaddrl);
	regmap_field_read(win->yaddrh, &yaddrh);
	yaddr = (yaddrh << 16 | yaddrl) << ADDR_SHIFT;

	regmap_field_read(win->caddrl, &caddrl);
	regmap_field_read(win->caddrh, &caddrh);
	caddr = (caddrh << 16 | caddrl) << ADDR_SHIFT;

	regmap_field_read(win->hst, &hst);
	regmap_field_read(win->hend, &hend);
	regmap_field_read(win->vst, &vst);
	regmap_field_read(win->vend, &vend);
	regmap_field_read(win->pitch, &pitch);
	regmap_field_read(win->src_width, &srcw);
	regmap_field_read(win->src_height, &srch);
	regmap_field_read(win->scale_h, &scaleh);
	regmap_field_read(win->scale_v, &scalev);

	dev_info(dev, "Window dump\n"
		      "enabled: %d\n"
		      "yaddr: 0x%08x\n"
		      "caddr: 0x%08x\n"
		      "horizontal start: %d, end %d\n"
		      "vertical start: %d, end %d\n"
		      "pitch: %d\n"
		      "source width: %d, height: %d\n"
		      "scale horizontal: %d, vertical: %d\n",
		      en,
		      yaddr,
		      caddr,
		      hst, hend,
		      vst, vend,
		      pitch,
		      srcw, srch,
		      scaleh, scalev);
}

static int mstar_mop_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	unsigned int hsize, vsize;
	struct mstar_mop *mop;
	struct regmap *regmap;
	void __iomem *base;
	int i;


	mop = devm_kzalloc(dev, sizeof(*mop), GFP_KERNEL);
	if (!mop)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(dev, base, &mstar_mop_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	mop->swrst = devm_regmap_field_alloc(dev, regmap, swrst_field);
	mop->gw_hsize = devm_regmap_field_alloc(dev, regmap, gw_hsize_field);
	mop->gw_vsize = devm_regmap_field_alloc(dev, regmap, gw_vsize_field);

	regmap_field_read(mop->gw_hsize, &hsize);
	regmap_field_read(mop->gw_vsize, &vsize);
	dev_info(dev, "MStar MOP\n"
		      "global window size; height: %d, width: %d\n",
		      hsize, vsize
		);

	for (i = 0; i < ARRAY_SIZE(mop->windows); i++){
		unsigned int offset = WINDOWS_START + (WINDOW_LEN * i);
		struct reg_field en_field = REG_FIELD(offset + 0, 0, 0);
		struct reg_field yaddrl_field = REG_FIELD(offset + 0x8, 0, 15);
		struct reg_field yaddrh_field = REG_FIELD(offset + 0xc, 0, 11);
		struct reg_field caddrl_field = REG_FIELD(offset + 0x10, 0, 15);
		struct reg_field caddrh_field = REG_FIELD(offset + 0x14, 0, 11);
		struct reg_field hst_field = REG_FIELD(offset + 0x18, 0, 12);
		struct reg_field hend_field = REG_FIELD(offset + 0x1c, 0, 12);
		struct reg_field vst_field = REG_FIELD(offset + 0x20, 0, 12);
		struct reg_field vend_field = REG_FIELD(offset + 0x24, 0, 12);
		struct reg_field pitch_field = REG_FIELD(offset + 0x28, 0, 12);
		struct reg_field srcw_field = REG_FIELD(offset + 0x2c, 0, 12);
		struct reg_field srch_field = REG_FIELD(offset + 0x30, 0, 12);
		struct reg_field scaleh_field = REG_FIELD(offset + 0x34, 0, 12);
		struct reg_field scalev_field = REG_FIELD(offset + 0x38, 0, 12);
		struct mstar_mop_window *window = &mop->windows[i];

		window->en = devm_regmap_field_alloc(dev, regmap, en_field);
		window->yaddrl = devm_regmap_field_alloc(dev, regmap, yaddrl_field);
		window->yaddrh = devm_regmap_field_alloc(dev, regmap, yaddrh_field);
		window->caddrl = devm_regmap_field_alloc(dev, regmap, caddrl_field);
		window->caddrh = devm_regmap_field_alloc(dev, regmap, caddrh_field);
		window->hst = devm_regmap_field_alloc(dev, regmap, hst_field);
		window->hend = devm_regmap_field_alloc(dev, regmap, hend_field);
		window->vst = devm_regmap_field_alloc(dev, regmap, vst_field);
		window->vend = devm_regmap_field_alloc(dev, regmap, vend_field);
		window->pitch = devm_regmap_field_alloc(dev, regmap, pitch_field);
		window->src_height = devm_regmap_field_alloc(dev, regmap, srcw_field);
		window->src_width = devm_regmap_field_alloc(dev, regmap, srch_field);
		window->scale_h = devm_regmap_field_alloc(dev, regmap, scaleh_field);
		window->scale_v = devm_regmap_field_alloc(dev, regmap, scalev_field);

		mstar_mop_dump_window(dev, window);
	}

	return 0;
}

static int mstar_mop_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id mstar_mop_ids[] = {
	{ .compatible = "sstar,ssd20xd-mop" },
	{},
};
MODULE_DEVICE_TABLE(of, mstar_mop_ids);

static struct platform_driver mstar_mop_driver = {
	.probe = mstar_mop_probe,
	.remove = mstar_mop_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = mstar_mop_ids,
	},
};

module_platform_driver(mstar_mop_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME);
MODULE_AUTHOR("Daniel Palmer <daniel@0x0f.com>");
