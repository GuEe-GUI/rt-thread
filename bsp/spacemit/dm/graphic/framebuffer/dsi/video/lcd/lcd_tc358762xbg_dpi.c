// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Spacemit Co., Ltd.
 *
 */

#include <linux/kernel.h>
#include "../../include/spacemit_dsi_common.h"
#include "../../include/spacemit_video_tx.h"
#include <linux/delay.h>

#define UNLOCK_DELAY 0

struct spacemit_mode_modeinfo tc358762xbg_dpi_800x480_spacemit_modelist[] = {
	{
		.name = "800x480-60",
		.refresh = 60,
		.xres = 800,
		.yres = 480,
		.real_xres = 800,
		.real_yres = 480,
		.left_margin = 1,
		.right_margin = 46,
		.hsync_len = 2,
		.upper_margin = 7,
		.lower_margin = 21,
		.vsync_len = 2,
		.hsync_invert = 0,
		.vsync_invert = 0,
		.invert_pixclock = 0,
		.pixclock_freq = 26*1000,
		.pix_fmt_out = OUTFMT_RGB888,
		.width = 95,
		.height = 53,
	},
};

struct spacemit_mipi_info tc358762xbg_dpi_800x480_mipi_info = {
	.height = 480,
	.width = 800,
	.hfp = 1, /* unit: pixel */
	.hbp = 46,
	.hsync = 2,
	.vfp = 7, /* unit: line */
	.vbp = 21,
	.vsync = 2,
	.fps = 60,

	.work_mode = SPACEMIT_DSI_MODE_VIDEO, /*command_mode, video_mode*/
	.rgb_mode = DSI_INPUT_DATA_RGB_MODE_888,
	.lane_number = 1,
	.phy_bit_clock = 700000000,
	.phy_esc_clock = 51200000,
	.split_enable = 0,
	.eotp_enable = 1,

	.burst_mode = DSI_BURST_MODE_BURST,
};

static struct spacemit_dsi_cmd_desc tc358762xbg_dpi_800x480_set_id_cmds[] = {

};

static struct spacemit_dsi_cmd_desc tc358762xbg_dpi_800x480_read_id_cmds[] = {

};

static struct spacemit_dsi_cmd_desc tc358762xbg_dpi_800x480_set_power_cmds[] = {

};

static struct spacemit_dsi_cmd_desc tc358762xbg_dpi_800x480_read_power_cmds[] = {

};

static struct spacemit_dsi_cmd_desc tc358762xbg_dpi_800x480_init_cmds[] = {
{SPACEMIT_DSI_DCS_LWRITE, SPACEMIT_DSI_LP_MODE, 0, 6, {0x02, 0x10, 0x00, 0x00, 0x00, 0x03}},
{SPACEMIT_DSI_DCS_LWRITE, SPACEMIT_DSI_LP_MODE, 0, 6, {0x01, 0x64, 0x00, 0x00, 0x00, 0x05}},
{SPACEMIT_DSI_DCS_LWRITE, SPACEMIT_DSI_LP_MODE, 0, 6, {0x01, 0x68, 0x00, 0x00, 0x00, 0x05}},
{SPACEMIT_DSI_DCS_LWRITE, SPACEMIT_DSI_LP_MODE, 0, 6, {0x01, 0x44, 0x00, 0x00, 0x00, 0x00}},
{SPACEMIT_DSI_DCS_LWRITE, SPACEMIT_DSI_LP_MODE, 0, 6, {0x01, 0x48, 0x00, 0x00, 0x00, 0x00}},
{SPACEMIT_DSI_DCS_LWRITE, SPACEMIT_DSI_LP_MODE, 0, 6, {0x01, 0x14, 0x00, 0x00, 0x00, 0x03}},
{SPACEMIT_DSI_DCS_LWRITE, SPACEMIT_DSI_LP_MODE, 0, 6, {0x04, 0x50, 0x00, 0x00, 0x00, 0x00}},
{SPACEMIT_DSI_DCS_LWRITE, SPACEMIT_DSI_LP_MODE, 0, 6, {0x04, 0x20, 0x00, 0x10, 0x01, 0x50}},
{SPACEMIT_DSI_DCS_LWRITE, SPACEMIT_DSI_LP_MODE, 100, 6, {0x04, 0x64, 0x00, 0x00, 0x04, 0x0f}},
{SPACEMIT_DSI_DCS_LWRITE, SPACEMIT_DSI_LP_MODE, 0, 6, {0x01, 0x04, 0x00, 0x00, 0x00, 0x01}},
{SPACEMIT_DSI_DCS_LWRITE, SPACEMIT_DSI_LP_MODE, 100, 6, {0x02, 0x04, 0x00, 0x00, 0x00, 0x01}},
};

static struct spacemit_dsi_cmd_desc tc358762xbg_dpi_800x480_sleep_out_cmds[] = {
	{SPACEMIT_DSI_DCS_SWRITE,SPACEMIT_DSI_LP_MODE,200,1,{0x11}},
	{SPACEMIT_DSI_DCS_SWRITE,SPACEMIT_DSI_LP_MODE,50,1,{0x29}},
};

static struct spacemit_dsi_cmd_desc tc358762xbg_dpi_800x480_sleep_in_cmds[] = {
	{SPACEMIT_DSI_DCS_SWRITE,SPACEMIT_DSI_LP_MODE,50,1,{0x28}},
	{SPACEMIT_DSI_DCS_SWRITE,SPACEMIT_DSI_LP_MODE,200,1,{0x10}},
};

struct lcd_mipi_panel_info lcd_tc358762xbg_dpi_800x480 = {
	.lcd_name = "lcd_tc358762xbg_dpi",
	.lcd_id = 0x00,
	.panel_id0 = 0x00,
	.power_value = 0x00,
	.panel_type = LCD_DPI,
	.width_mm = 95,
	.height_mm = 53,
	.dft_pwm_bl = 128,
	.set_id_cmds_num = ARRAY_SIZE(tc358762xbg_dpi_800x480_set_id_cmds),
	.read_id_cmds_num = ARRAY_SIZE(tc358762xbg_dpi_800x480_read_id_cmds),
	.init_cmds_num = ARRAY_SIZE(tc358762xbg_dpi_800x480_init_cmds),
	.set_power_cmds_num = ARRAY_SIZE(tc358762xbg_dpi_800x480_set_power_cmds),
	.read_power_cmds_num = ARRAY_SIZE(tc358762xbg_dpi_800x480_read_power_cmds),
	.sleep_out_cmds_num = ARRAY_SIZE(tc358762xbg_dpi_800x480_sleep_out_cmds),
	.sleep_in_cmds_num = ARRAY_SIZE(tc358762xbg_dpi_800x480_sleep_in_cmds),
	//.drm_modeinfo = tc358762xbg_dpi_800x480_modelist,
	.spacemit_modeinfo = tc358762xbg_dpi_800x480_spacemit_modelist,
	.mipi_info = &tc358762xbg_dpi_800x480_mipi_info,
	.set_id_cmds = tc358762xbg_dpi_800x480_set_id_cmds,
	.read_id_cmds = tc358762xbg_dpi_800x480_read_id_cmds,
	.set_power_cmds = tc358762xbg_dpi_800x480_set_power_cmds,
	.read_power_cmds = tc358762xbg_dpi_800x480_read_power_cmds,
	.init_cmds = tc358762xbg_dpi_800x480_init_cmds,
	.sleep_out_cmds = tc358762xbg_dpi_800x480_sleep_out_cmds,
	.sleep_in_cmds = tc358762xbg_dpi_800x480_sleep_in_cmds,
	.bitclk_sel = 3,
	.bitclk_div = 1,
	.pxclk_sel = 2,
	.pxclk_div = 6,
};

int lcd_tc358762xbg_dpi_800x480_init(void)
{
	int ret;

	ret = lcd_mipi_register_panel(&lcd_tc358762xbg_dpi_800x480);
	return ret;
}
