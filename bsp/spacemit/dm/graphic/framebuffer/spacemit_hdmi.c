// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Spacemit Co., Ltd.
 *
 */

#include <asm/gpio.h>
#include <asm/io.h>
#include <common.h>
#include <clk.h>
#include <display.h>
#include <dm.h>
#include <dw_hdmi.h>
#include <edid.h>
#include <regmap.h>
#include <syscon.h>

#include <power-domain-uclass.h>
#include <power-domain.h>
#include <power/regulator.h>
#include <linux/delay.h>
#include <linux/io.h>

#include "spacemit_hdmi.h"

#define SPACEMIT_HDMI_PHY_STATUS        0xC
#define SPACEMIT_HDMI_PHY_HPD           0x1000
extern bool is_video_connected;

static int hdmi_get_plug_in_status(struct dw_hdmi *hdmi)
{
	void __iomem *hdmi_addr;
	hdmi_addr = ioremap(0xC0400500, 0x200);
	u32 value;

	pr_debug("%s() \n", __func__);
	value = readl(hdmi_addr + SPACEMIT_HDMI_PHY_STATUS) & SPACEMIT_HDMI_PHY_HPD;

	return !!value;
}

static int hdmi_phy_wait_for_hpd(struct dw_hdmi *hdmi)
{
	ulong start;

	pr_debug("%s() \n", __func__);

	start = get_timer(0);
	do {
		if (hdmi_get_plug_in_status(hdmi)) {
			pr_info("%s() hdmi get hpd signal \n", __func__);
			return 0;
		}
		udelay(100);
	} while (get_timer(start) < 100);

	return -1;
}

enum bit_depth{
	EIGHT_BPP = 0,
	TEN_BPP = 1,
	TWELVE_BPP =2,
};

int power_of_two(int n) {
	int result = 1;
	for (int i = 0; i < n; ++i) {
		result <<= 1;
	}

	return result;
}

int pll8_bit_5_6 (int bit_clock, int n){
	int ret = 0;
	bit_clock = bit_clock / n;

	if (bit_clock < 425)
		ret = 3;
	else if (bit_clock < 850)
		ret = 2;
	else if (bit_clock < 1700)
		ret = 1;
	else
		ret = 0;

	return ret;
}

int pll6_bit_4_5 (int bit_clock, int n){
	int ret = 0;
	bit_clock = bit_clock / n;

	if (bit_clock <= 337)
		ret = 0;
	else if (bit_clock < 425)
		ret = 1;
	else if (bit_clock < 675)
		ret = 0;
	else if (bit_clock < 850)
		ret = 1;
	else if (bit_clock < 1350)
		ret = 0;
	else if (bit_clock < 1700)
		ret = 1;
	else
		ret = 0;

	return ret;
}

int pll5_bit_0_2 (int bit_clock, int n){
	int value =  bit_clock * power_of_two(pll8_bit_5_6(bit_clock, n)) / n;
	int ret;

	if (value < 1830)
		ret = 0;
	else if (value < 2030)
		ret = 1;
	else if (value < 2275)
		ret = 2;
	else if (value < 2520)
		ret = 3;
	else if (value < 2765)
		ret = 4;
	else if (value < 3015)
		ret = 5;
	else if (value < 3260)
		ret = 6;
	else
		ret = 7;

	return ret;
}

int PLL9_BIT0_1[3] = {0x0, 0x1, 0x2};

void pll_reg_cal(int bit_clock, int ref_clock, int n, int *integer_part, u32 *hmdi_e8_reg) {
	long long int_para = 1000000000;
	long long value = (power_of_two(pll8_bit_5_6(bit_clock, n))) * bit_clock * int_para / (n * (pll6_bit_4_5(bit_clock, n) + 1) * ref_clock);
	long long integer = (power_of_two(pll8_bit_5_6(bit_clock, n)))* bit_clock / (n * (pll6_bit_4_5(bit_clock, n) + 1) * ref_clock) * int_para;
	long long fraction = value - integer;
	bool negative = false;
	int bit = 0;
	int frac_20bit = 0;
	int pll2_reg = 0;
	int pll1_reg = 0;
	int pll0_reg = 0;

	negative =  fraction > 500000000 ? true : false;
	fraction = negative ? 1000000000 - fraction : fraction;
	*integer_part = negative ? integer/int_para + 1 : integer/int_para;


	for (int i = 0; i < 20; i++){
		if (bit >= int_para) {
			frac_20bit |= 1 << (19 - i);
			fraction -= int_para;
		}
		fraction *= 2;
		bit = fraction;
	}

	if (!negative){
		pll2_reg = ((frac_20bit & 0xF0000) >> 16) | (1 << 5);
	} else {
		frac_20bit = (~frac_20bit + 1) & 0xfffff;
		pll2_reg = 0x10 | ((frac_20bit & 0xF0000) >> 16) | (1 << 5);
	}

	pll1_reg = (frac_20bit & 0xFF00) >> 8;
	pll0_reg = frac_20bit & 0xFF;
	*hmdi_e8_reg = (0x20 << 24) | (pll2_reg << 16) | (pll1_reg << 8) | pll0_reg;
}

int pll_reg (void __iomem *hdmi_addr, int pixel_clock, int bit_depth) {
	int pll9_reg = 0, pll8_reg = 0, pll7_reg = 0, pll6_reg = 0, pll5_reg = 0, pll4_reg = 0;
	int n = 100;
	int ref_clock = 24;
	int hdmi_ec_reg = 0;
	int hdmi_f0_reg = 0;
	int hdmi_e8_reg = 0;
	int pow = 0;
	int bit_clock = bit_depth == EIGHT_BPP ? pixel_clock : pixel_clock * 125 / 100;

	int integer_part = 0;

	pll_reg_cal(bit_clock, ref_clock, n, &integer_part, &hdmi_e8_reg);

	pll9_reg = 2 << 2 | PLL9_BIT0_1[bit_depth];
	pll8_reg = (0 << 7) | (pll8_bit_5_6(bit_clock, n) << 5) | 1;
	pll7_reg = 0x50;
	pll6_reg = 0xD | (pll6_bit_4_5(bit_clock, n) << 4) | (2 << 6);
	pll5_reg = 0x40 | pll5_bit_0_2(bit_clock, n);

	pow = (pll8_bit_5_6(bit_clock, n));

	pll4_reg = integer_part;

	hdmi_ec_reg = (pll7_reg << 24) | (pll6_reg << 16) | (pll5_reg << 8) | pll4_reg;
	hdmi_f0_reg = (pll9_reg << 8) | pll8_reg;

	writel(hdmi_e8_reg, hdmi_addr + 0xe8);
	writel(hdmi_ec_reg, hdmi_addr + 0xec);
	writel(hdmi_f0_reg, hdmi_addr + 0xf0);

	return 0;
}

void hdmi_write_bits(void __iomem *hdmi_addr, u16 offset, u32 value, u32 mask, u32 shifts)
{
	u32 reg_val;

	reg_val = readl(hdmi_addr + (offset));
	reg_val &= ~(mask << shifts);
	reg_val |= (value << shifts);
	writel(reg_val, hdmi_addr + (offset));
}

void hdmi_init (void __iomem *hdmi_addr, int pixel_clock, int bit_depth){
	u32 value = 0;
	int color_depth = bit_depth == EIGHT_BPP ? 4 : 5;

	u32 good_phase = 0x00;
	u32 bias_current = 0x01;
	u32 bias_risistor = 0x07;

	// hdmi phy config
	writel(0x20200000, hdmi_addr + 0xe8);
	writel(0x508d425a, hdmi_addr + 0xec);
	writel(0x861, hdmi_addr + 0xf0);

	writel(0xAE5C410F, hdmi_addr + 0xe0);
	hdmi_write_bits(hdmi_addr, 0xe0, bias_current, 0x03, 29);
	hdmi_write_bits(hdmi_addr, 0xe0, bias_risistor, 0x0F, 18);
	hdmi_write_bits(hdmi_addr, 0xe0, good_phase, 0x03, 14);

	value = 0x0000000d | (color_depth << 4);
	writel(value, hdmi_addr + 0x34);

	pll_reg(hdmi_addr, pixel_clock, bit_depth);
	writel(0x00, hdmi_addr + 0xe4);
	writel(0x03, hdmi_addr + 0xe4);
	value = readl(hdmi_addr + 0xe4);

	pr_debug("%s() hdmi pll lock status 0x%x\n", __func__, value);
	// while ( (value & 0x10000) != 0) {
	// 	value = readl(hdmi->regs + 0xe4);
	// }
	udelay(100);

	value = 0x1C208000 | bit_depth;
	writel(value, hdmi_addr + 0x28);
}


static int hdmi_enable(struct udevice *dev, int panel_bpp,
			      const struct display_timing *edid)
{
	void __iomem *hdmi_addr;
	hdmi_addr = ioremap(0xC0400500, 0x200);
	u32 pixel_clock = 148500;
	int bit_depth = EIGHT_BPP;

	hdmi_init(hdmi_addr, pixel_clock, bit_depth);

	return 0;
}

int hdmi_read_edid(struct udevice *dev, u8 *buf, int buf_size)
{
	struct spacemit_hdmi_priv *priv = dev_get_priv(dev);

	return dw_hdmi_read_edid(&priv->hdmi, buf, buf_size);
}

static int spacemit_hdmi_of_to_plat(struct udevice *dev)
{
	return 0;
}

static int spacemit_hdmi_probe(struct udevice *dev)
{
	struct spacemit_hdmi_priv *priv = dev_get_priv(dev);
	struct power_domain pm_domain;
	unsigned long rate;
	int ret;

	pr_debug("%s() \n", __func__);

	priv->base = dev_remap_addr_name(dev, "hdmi");
	if (!priv->base)
		return -EINVAL;

	ret = power_domain_get(dev, &pm_domain);
	if (ret) {
		pr_err("power_domain_get hdmi failed: %d", ret);
		return ret;
	}

	ret = clk_get_by_name(dev, "hmclk", &priv->hdmi_mclk);
	if (ret) {
		pr_err("clk_get_by_name hdmi mclk failed: %d", ret);
		return ret;
	}

	ret = reset_get_by_name(dev, "hdmi_reset", &priv->hdmi_reset);
	if (ret) {
		pr_err("reset_get_by_name hdmi reset failed: %d\n", ret);
		return ret;
	}

	ret = reset_deassert(&priv->hdmi_reset);
	if (ret) {
		pr_err("reset_assert hdmi reset failed: %d\n", ret);
		return ret;
	}

	ret = clk_enable(&priv->hdmi_mclk);
	if (ret < 0) {
		pr_err("clk_enable hdmi mclk failed: %d\n", ret);
		return ret;
	}

	ret = clk_set_rate(&priv->hdmi_mclk, 491520000);
	if (ret < 0) {
		pr_err("clk_set_rate mipi dsi mclk failed: %d\n", ret);
		return ret;
	}

	rate = clk_get_rate(&priv->hdmi_mclk);
	pr_debug("%s clk_get_rate hdmi mclk %ld\n", __func__, rate);

	priv->hdmi.ioaddr = (ulong)priv->base;
	priv->hdmi.reg_io_width = 4;

	ret = hdmi_phy_wait_for_hpd(&priv->hdmi);
	is_video_connected = (ret >= 0);
	if (!is_video_connected) {
		pr_info("HDMI cannot get HPD signal\n");
		return ret;
	}

	return 0;
}

static const struct dm_display_ops spacemit_hdmi_ops = {
	.read_edid = hdmi_read_edid,
	.enable = hdmi_enable,
};

static const struct udevice_id spacemit_hdmi_ids[] = {
	{ .compatible = "spacemit,hdmi" },
	{ }
};

U_BOOT_DRIVER(spacemit_hdmi) = {
	.name = "spacemit_hdmi",
	.id = UCLASS_DISPLAY,
	.of_match = spacemit_hdmi_ids,
	.ops = &spacemit_hdmi_ops,
	.of_to_plat = spacemit_hdmi_of_to_plat,
	.probe = spacemit_hdmi_probe,
	.priv_auto	= sizeof(struct spacemit_hdmi_priv),
};
