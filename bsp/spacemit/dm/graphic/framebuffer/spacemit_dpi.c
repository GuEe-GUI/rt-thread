// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2025 Spacemit Co., Ltd.
 *
 */

#include <common.h>
#include <backlight.h>
#include <dm.h>
#include <mipi_dsi.h>
#include <panel.h>
#include <asm/gpio.h>
#include <dm/device_compat.h>
#include <linux/delay.h>
#include <power/regulator.h>
#include <i2c.h>
#include <asm/arch/gpio.h>
#include <asm/io.h>
#include "./dsi/include/spacemit_dsi_common.h"
#include "./dsi/include/spacemit_video_tx.h"


#define RPI_DSI_DRIVER_NAME "rpi-ts-dsi"

struct dpi_panel_priv {
	struct udevice *reg;
	struct udevice *backlight;

	struct gpio_desc enable_gpio;
};

/* I2C registers of the Atmel microcontroller. */
enum REG_ADDR {
	REG_ID = 0x80,
	REG_PORTA, /* BIT(2) for horizontal flip, BIT(3) for vertical flip */
	REG_PORTB,
	REG_PORTC,
	REG_PORTD,
	REG_POWERON,
	REG_PWM,
	REG_DDRA,
	REG_DDRB,
	REG_DDRC,
	REG_DDRD,
	REG_TEST,
	REG_WR_ADDRL,
	REG_WR_ADDRH,
	REG_READH,
	REG_READL,
	REG_WRITEH,
	REG_WRITEL,
	REG_ID2,
};

/* DSI D-PHY Layer Registers */
#define D0W_DPHYCONTTX		0x0004
#define CLW_DPHYCONTRX		0x0020
#define D0W_DPHYCONTRX		0x0024
#define D1W_DPHYCONTRX		0x0028
#define COM_DPHYCONTRX		0x0038
#define CLW_CNTRL		0x0040
#define D0W_CNTRL		0x0044
#define D1W_CNTRL		0x0048
#define DFTMODE_CNTRL		0x0054

/* DSI PPI Layer Registers */
#define PPI_STARTPPI		0x0104
#define PPI_BUSYPPI		0x0108
#define PPI_LINEINITCNT		0x0110
#define PPI_LPTXTIMECNT		0x0114
#define PPI_CLS_ATMR		0x0140
#define PPI_D0S_ATMR		0x0144
#define PPI_D1S_ATMR		0x0148
#define PPI_D0S_CLRSIPOCOUNT	0x0164
#define PPI_D1S_CLRSIPOCOUNT	0x0168
#define CLS_PRE			0x0180
#define D0S_PRE			0x0184
#define D1S_PRE			0x0188
#define CLS_PREP		0x01A0
#define D0S_PREP		0x01A4
#define D1S_PREP		0x01A8
#define CLS_ZERO		0x01C0
#define D0S_ZERO		0x01C4
#define D1S_ZERO		0x01C8
#define PPI_CLRFLG		0x01E0
#define PPI_CLRSIPO		0x01E4
#define HSTIMEOUT		0x01F0
#define HSTIMEOUTENABLE		0x01F4

/* DSI Protocol Layer Registers */
#define DSI_STARTDSI		0x0204
#define DSI_BUSYDSI		0x0208
#define DSI_LANEENABLE		0x0210
# define DSI_LANEENABLE_CLOCK		BIT(0)
# define DSI_LANEENABLE_D0		BIT(1)
# define DSI_LANEENABLE_D1		BIT(2)

#define DSI_LANESTATUS0		0x0214
#define DSI_LANESTATUS1		0x0218
#define DSI_INTSTATUS		0x0220
#define DSI_INTMASK		0x0224
#define DSI_INTCLR		0x0228
#define DSI_LPTXTO		0x0230
#define DSI_MODE		0x0260
#define DSI_PAYLOAD0		0x0268
#define DSI_PAYLOAD1		0x026C
#define DSI_SHORTPKTDAT		0x0270
#define DSI_SHORTPKTREQ		0x0274
#define DSI_BTASTA		0x0278
#define DSI_BTACLR		0x027C

/* DSI General Registers */
#define DSIERRCNT		0x0300
#define DSISIGMOD		0x0304

/* DSI Application Layer Registers */
#define APLCTRL			0x0400
#define APLSTAT			0x0404
#define APLERR			0x0408
#define PWRMOD			0x040C
#define RDPKTLN			0x0410
#define PXLFMT			0x0414
#define MEMWRCMD		0x0418

/* LCDC/DPI Host Registers */
#define LCDCTRL			0x0420
#define HSR			0x0424
#define HDISPR			0x0428
#define VSR			0x042C
#define VDISPR			0x0430
#define VFUEN			0x0434

/* DBI-B Host Registers */
#define DBIBCTRL		0x0440

/* SPI Master Registers */
#define SPICMR			0x0450
#define SPITCR			0x0454

/* System Controller Registers */
#define SYSSTAT			0x0460
#define SYSCTRL			0x0464
#define SYSPLL1			0x0468
#define SYSPLL2			0x046C
#define SYSPLL3			0x0470
#define SYSPMCTRL		0x047C

/* GPIO Registers */
#define GPIOC			0x0480
#define GPIOO			0x0484
#define GPIOI			0x0488

/* I2C Registers */
#define I2CCLKCTRL		0x0490

/* Chip/Rev Registers */
#define IDREG			0x04A0

/* Debug Registers */
#define WCMDQUEUE		0x0500
#define RCMDQUEUE		0x0504


static const struct display_timing default_timing = {
	.pixelclock.typ		= 26000000,
	.hactive.typ		= 800,
	.hfront_porch.typ	= 1,
	.hback_porch.typ	= 46,
	.hsync_len.typ		= 2,
	.vactive.typ		= 480,
	.vfront_porch.typ	= 7,
	.vback_porch.typ	= 21,
	.vsync_len.typ		= 2,
};

static int rpi_touchscreen_i2c_read(struct udevice *dev, uint8_t addr, uint8_t *data)
{
	uint8_t valb;
	int err;

	err = dm_i2c_read(dev, addr, &valb, 1);
	if (err)
		return err;

	*data = (int)valb;
	return 0;
}

static int rpi_touchscreen_i2c_write(struct udevice *dev, uint addr, uint8_t data)
{
	uint8_t valb;
	int err;
	valb = data;

	err = dm_i2c_write(dev, addr, &valb, 1);
	return err;
}

static int rpi_touchscreen_prepare(struct udevice *dev)
{
	int i, ret = 0;
	uint8_t data;
	struct video_tx_device *tx_device = NULL;
	struct lcd_mipi_tx_data  *video_tx_client;
	rpi_touchscreen_i2c_write(dev, REG_POWERON, 1);
	/* Wait for nPWRDWN to go low to indicate poweron is done. */
	for (i = 0; i < 100; i++) {
		rpi_touchscreen_i2c_read(dev, REG_PORTB, &data);
		if (data & 1)
			break;
	}
	tx_device = find_video_tx();
	video_tx_client = video_tx_get_drvdata(tx_device);

	//initcmd
	ret = spacemit_mipi_write_cmds(0, video_tx_client->panel_info->init_cmds,
			video_tx_client->panel_info->init_cmds_num);
	if(ret) {
			pr_info("rpi touchscreen send init cmds fail!\n ");
		}
	return 0;
}

static int dpi_panel_enable_backlight(struct udevice *dev)
{
	pr_debug("%s: device %s \n", __func__, dev->name);

	rpi_touchscreen_prepare(dev);
	mdelay(100);
	/* Turn on the backlight. */
	rpi_touchscreen_i2c_write(dev, REG_PWM, 255);

	/* Default to the same orientation as the closed source
	 * firmware used for the panel.  Runtime rotation
	 * configuration will be supported using VC4's plane
	 * orientation bits.
	 */
	rpi_touchscreen_i2c_write(dev, REG_PORTA, BIT(2));

	return 0;
}

static int dpi_panel_get_display_timing(struct udevice *dev,
					    struct display_timing *timings)
{
	pr_debug("%s: device %s \n", __func__, dev->name);
	memcpy(timings, &default_timing, sizeof(*timings));
	return 0;
}

static int dpi_panel_of_to_plat(struct udevice *dev)
{
	pr_debug("%s: device %s \n", __func__, dev->name);
	return 0;
}

static int dpi_panel_probe(struct udevice *dev)
{
	uint8_t ver;
	int ret = 0;

	struct dpi_panel_priv *priv = dev_get_priv(dev);

	ret = gpio_request_by_name(dev, "enable-gpios", 0, &priv->enable_gpio,
				GPIOD_IS_OUT);
	if (ret) {
		pr_info("%s: Warning: cannot get enable GPIO: ret=%d\n",
		      __func__, ret);
	}
	dm_gpio_set_value(&priv->enable_gpio, 1);
	mdelay(100);

	ret = rpi_touchscreen_i2c_read(dev, REG_ID, &ver);
	if (ret < 0) {
		pr_info("%s: Atmel I2C read failed: %d\n", __func__, ver);
		return -1;
	}

	switch (ver) {
	case 0xde: /* ver 1 */
	case 0xc3: /* ver 2 */
		break;
	default:
		pr_info("%s: Unknown Atmel firmware revision: 0x%02x\n", __func__, ver);
		return -1;
	}

	/* Turn off at boot, so we can cleanly sequence powering on. */
	rpi_touchscreen_i2c_write(dev, REG_POWERON, 0);

	return 0;
}


static const struct panel_ops dpi_panel_ops = {
	.enable_backlight = dpi_panel_enable_backlight,
	.get_display_timing = dpi_panel_get_display_timing,
};

static const struct udevice_id dpi_panel_ids[] = {
	{ .compatible = "raspberrypi,7inch-touchscreen-panel" },
	{ }
};


U_BOOT_DRIVER(dpi_panel) = {
	.name			= "dpi_panel",
	.id			= UCLASS_PANEL,
	.of_match		= dpi_panel_ids,
	.ops			= &dpi_panel_ops,
	.of_to_plat		= dpi_panel_of_to_plat,
	.probe			= dpi_panel_probe,
	.plat_auto		= sizeof(struct mipi_dsi_panel_plat),
	.priv_auto		= sizeof(struct dpi_panel_priv),
};
