// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Daniel Palmer<daniel@thingy.jp>
 */

#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/of_irq.h>
#include <linux/dmaengine.h>
#include <linux/of_dma.h>

#define DRIVER_NAME "ssd20xd-movedma"
#define CHANNELS 1

#define REG_EN			0x0
#define REG_SRC_START_ADDR_L	0xc
#define REG_SRC_START_ADDR_H	0x10
#define REG_BYTE_CNT_L		0x1c
#define REG_BYTE_CNT_H		0x20
#define REG_DMAMODE		0x144
#define REG_DEVSEL		0x148

static struct reg_field en_field = REG_FIELD(REG_EN, 0, 0);
static struct reg_field dmamode_field = REG_FIELD(REG_DMAMODE, 0, 0);
#define DMAMODE_DEVICE	1
static struct reg_field devsel_field = REG_FIELD(REG_DEVSEL, 0, 0);

static const struct regmap_config ssd20xd_movedma_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.reg_stride = 4
};

struct ssd20xd_movedma {
	struct dma_device dma_device;
	struct clk *clk;
	int irq;

	struct regmap *regmap;
	struct regmap_field *en;
	struct regmap_field *dmamode;
	struct regmap_field *devsel;
};

struct ssd20xd_movedma_chan {
	struct dma_chan chan;
	struct list_head queue;
	dma_cookie_t cookie;
};

#define to_chan(ch) container_of(ch, struct ssd20xd_movedma_chan, chan);

struct ssd20xd_movedma_desc {
	struct dma_async_tx_descriptor tx;
	size_t len;
	dma_addr_t dst;
	dma_addr_t src;
	struct list_head queue_node;
};

#define to_desc(desc) container_of(desc, struct ssd20xd_movedma_desc, tx);

static irqreturn_t ssd20xd_movedma_irq(int irq, void *data)
{
	struct ssd20xd_movedma *movedma = data;

	printk("%s:%d\n", __func__, __LINE__);

	return IRQ_HANDLED;
}

static enum dma_status ssd20xd_movedma_tx_status(struct dma_chan *chan,
		dma_cookie_t cookie, struct dma_tx_state *txstate)
{
	printk("%s:%d\n", __func__, __LINE__);
	return DMA_ERROR;
}

static void ssd20xd_movedma_do_single(struct ssd20xd_movedma_chan *chan, struct ssd20xd_movedma_desc *desc)
{
	printk("%s:%d\n", __func__, __LINE__);
}

static void ssd20xd_movedma_issue_pending(struct dma_chan *chan)
{
	struct ssd20xd_movedma_desc *desc;
	struct ssd20xd_movedma_chan *ch = to_chan(chan);

	printk("%s:%d\n", __func__, __LINE__);

	desc = list_first_entry_or_null(&ch->queue, struct ssd20xd_movedma_desc, queue_node);
	if(desc)
		ssd20xd_movedma_do_single(ch, desc);
}

static dma_cookie_t ssd20xd_movedma_tx_submit(struct dma_async_tx_descriptor *tx)
{
	struct ssd20xd_movedma_chan *chan = to_chan(tx->chan);
	struct ssd20xd_movedma_desc *desc = to_desc(tx);

	printk("%s:%d\n", __func__, __LINE__);

	list_add_tail(&desc->queue_node, &chan->queue);

	return chan->cookie++;
}

static struct dma_async_tx_descriptor* ssd20xd_movedma_prep_slave_sg(
		struct dma_chan *chan, struct scatterlist *sgl,
		unsigned int sg_len, enum dma_transfer_direction direction,
		unsigned long flags, void *context)
{
	struct ssd20xd_movedma_chan *ch;
	struct ssd20xd_movedma_desc *desc;
	u32 dmaaddr, dmalen;
	int width;

	if(sg_len != 1){
		printk("only one sg for now\n");
		return NULL;
	}

	ch = to_chan(chan);

	dmaaddr = sg_dma_address(sgl);
	dmalen = sg_dma_len(sgl);

	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	dma_async_tx_descriptor_init(&desc->tx, chan);

	desc->len = dmalen;

	switch(direction){
		case DMA_DEV_TO_MEM:

			break;
		case DMA_MEM_TO_DEV:
			break;
		default:
			goto free_desc;
	}

	desc->tx.tx_submit = ssd20xd_movedma_tx_submit;

	return &desc->tx;

free_desc:
	kfree(desc);
	return NULL;
}

static int ssd20xd_movedma_probe(struct platform_device *pdev)
{
	struct ssd20xd_movedma_chan *chan;
	struct ssd20xd_movedma *movedma;
	struct device *dev = &pdev->dev;
	struct resource *res;
	void __iomem *base;
	int i, ret;

	printk("movedma probe\n");

	movedma = devm_kzalloc(&pdev->dev, sizeof(*movedma), GFP_KERNEL);
	if (!movedma)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(base)) {
		return PTR_ERR(base);
	}

	movedma->irq = irq_of_parse_and_map(pdev->dev.of_node, i);
	if (!movedma->irq)
		return -EINVAL;
	ret = devm_request_irq(&pdev->dev, movedma->irq, ssd20xd_movedma_irq, IRQF_SHARED,
			dev_name(&pdev->dev), chan);

	//movedma->clk = devm_clk_get(&pdev->dev, NULL);
	//if (IS_ERR(movedma->clk)) {
	//	return PTR_ERR(movedma->clk);
	//}

	movedma->dma_device.dev = &pdev->dev;
	movedma->dma_device.device_tx_status = ssd20xd_movedma_tx_status;
	movedma->dma_device.device_issue_pending = ssd20xd_movedma_issue_pending;
	movedma->dma_device.src_addr_widths = BIT(4);
	movedma->dma_device.dst_addr_widths = BIT(4);
	movedma->dma_device.directions = BIT(DMA_MEM_TO_MEM) |
					 BIT(DMA_DEV_TO_MEM) |
					 BIT(DMA_MEM_TO_DEV);
	movedma->dma_device.device_prep_slave_sg = ssd20xd_movedma_prep_slave_sg;

	INIT_LIST_HEAD(&movedma->dma_device.channels);

	for(i = 0; i < CHANNELS; i++){
		chan = devm_kzalloc(&pdev->dev, sizeof(*chan), GFP_KERNEL);
		if (!chan)
			return -ENOMEM;

		INIT_LIST_HEAD(&chan->queue);

		chan->chan.device = &movedma->dma_device;

		list_add_tail(&chan->chan.device_node, &movedma->dma_device.channels);
	}

	movedma->regmap = devm_regmap_init_mmio(&pdev->dev, base, &ssd20xd_movedma_regmap_config);
	if (IS_ERR(movedma->regmap))
		return PTR_ERR(movedma->regmap);

	movedma->en = devm_regmap_field_alloc(dev, movedma->regmap, en_field);
	movedma->dmamode = devm_regmap_field_alloc(dev, movedma->regmap, dmamode_field);
	movedma->devsel = devm_regmap_field_alloc(dev, movedma->regmap, devsel_field);

	regmap_field_write(movedma->en, 1);
	regmap_field_write(movedma->dmamode, DMAMODE_DEVICE);
	regmap_field_write(movedma->devsel, 0);

	//ret = clk_prepare_enable(movedma->clk);
	//if (ret)
	//	return ret;

	ret = dma_async_device_register(&movedma->dma_device);
	if(ret)
		return ret;

	return of_dma_controller_register(pdev->dev.of_node,
			of_dma_xlate_by_chan_id, &movedma->dma_device);
}

static const struct of_device_id ssd20xd_movedma_of_match[] = {
	{ .compatible = "sstar,ssd20xd-movedma", },
	{},
};

static struct platform_driver ssd20xd_movedma_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.of_match_table = ssd20xd_movedma_of_match,
	},
	.probe = ssd20xd_movedma_probe,
};
builtin_platform_driver(ssd20xd_movedma_driver);
