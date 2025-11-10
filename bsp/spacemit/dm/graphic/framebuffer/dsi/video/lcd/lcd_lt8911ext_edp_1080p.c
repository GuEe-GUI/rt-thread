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

struct spacemit_mode_modeinfo lt8911ext_edp_1080p_spacemit_modelist[] = {
	{
		.name = "1920x1080-60",
		.refresh = 60,
		.xres = 1920,
		.yres = 1080,
		.real_xres = 1920,
		.real_yres = 1080,
		.left_margin = 200,
		.right_margin = 48,
		.hsync_len = 32,
		.upper_margin = 31,
		.lower_margin = 3,
		.vsync_len = 6,
		.hsync_invert = 0,
		.vsync_invert = 0,
		.invert_pixclock = 0,
		.pixclock_freq = 148*1000,
		.pix_fmt_out = OUTFMT_RGB888,
		.width = 309,
		.height = 174,
	},
};

struct spacemit_mipi_info lt8911ext_edp_1080p_mipi_info = {
	.height = 1080,
	.width = 1920,
	.hfp = 48, /* unit: pixel */
	.hbp = 200,
	.hsync = 32,
	.vfp = 3, /* unit: line */
	.vbp = 31,
	.vsync = 6,
	.fps = 60,

	.work_mode = SPACEMIT_DSI_MODE_VIDEO, /*command_mode, video_mode*/
	.rgb_mode = DSI_INPUT_DATA_RGB_MODE_888,
	.lane_number = 4,
	.phy_bit_clock = 933000000,
	.phy_esc_clock = 51200000,
	.split_enable = 0,
	.eotp_enable = 1,

	.burst_mode = DSI_BURST_MODE_BURST,
};

static struct spacemit_dsi_cmd_desc lt8911ext_edp_1080p_set_id_cmds[] = {

};

static struct spacemit_dsi_cmd_desc lt8911ext_edp_1080p_read_id_cmds[] = {

};

static struct spacemit_dsi_cmd_desc lt8911ext_edp_1080p_set_power_cmds[] = {

};

static struct spacemit_dsi_cmd_desc lt8911ext_edp_1080p_read_power_cmds[] = {

};

static struct spacemit_dsi_cmd_desc lt8911ext_edp_1080p_init_cmds[] = {
	{SPACEMIT_DSI_DCS_LWRITE, SPACEMIT_DSI_LP_MODE, 200, 2, {0x11, 0x00}},
	{SPACEMIT_DSI_DCS_LWRITE, SPACEMIT_DSI_LP_MODE,  50, 2, {0x29, 0x00}},
};

static struct spacemit_dsi_cmd_desc lt8911ext_edp_1080p_sleep_out_cmds[] = {
	{SPACEMIT_DSI_DCS_SWRITE,SPACEMIT_DSI_LP_MODE,200,1,{0x11}},
	{SPACEMIT_DSI_DCS_SWRITE,SPACEMIT_DSI_LP_MODE,50,1,{0x29}},
};

static struct spacemit_dsi_cmd_desc lt8911ext_edp_1080p_sleep_in_cmds[] = {
	{SPACEMIT_DSI_DCS_SWRITE,SPACEMIT_DSI_LP_MODE,50,1,{0x28}},
	{SPACEMIT_DSI_DCS_SWRITE,SPACEMIT_DSI_LP_MODE,200,1,{0x10}},
};


struct lcd_mipi_panel_info lcd_lt8911ext_edp_1080p = {
	.lcd_name = "lt8911ext_edp_1080p",
	.lcd_id = 0x00,
	.panel_id0 = 0x00,
	.power_value = 0x00,
	.panel_type = LCD_EDP,
	.width_mm = 309,
	.height_mm = 174,
	.dft_pwm_bl = 128,
	.set_id_cmds_num = ARRAY_SIZE(lt8911ext_edp_1080p_set_id_cmds),
	.read_id_cmds_num = ARRAY_SIZE(lt8911ext_edp_1080p_read_id_cmds),
	.init_cmds_num = ARRAY_SIZE(lt8911ext_edp_1080p_init_cmds),
	.set_power_cmds_num = ARRAY_SIZE(lt8911ext_edp_1080p_set_power_cmds),
	.read_power_cmds_num = ARRAY_SIZE(lt8911ext_edp_1080p_read_power_cmds),
	.sleep_out_cmds_num = ARRAY_SIZE(lt8911ext_edp_1080p_sleep_out_cmds),
	.sleep_in_cmds_num = ARRAY_SIZE(lt8911ext_edp_1080p_sleep_in_cmds),
	//.drm_modeinfo = lt8911ext_edp_1080p_modelist,
	.spacemit_modeinfo = lt8911ext_edp_1080p_spacemit_modelist,
	.mipi_info = &lt8911ext_edp_1080p_mipi_info,
	.set_id_cmds = lt8911ext_edp_1080p_set_id_cmds,
	.read_id_cmds = lt8911ext_edp_1080p_read_id_cmds,
	.set_power_cmds = lt8911ext_edp_1080p_set_power_cmds,
	.read_power_cmds = lt8911ext_edp_1080p_read_power_cmds,
	.init_cmds = lt8911ext_edp_1080p_init_cmds,
	.sleep_out_cmds = lt8911ext_edp_1080p_sleep_out_cmds,
	.sleep_in_cmds = lt8911ext_edp_1080p_sleep_in_cmds,
	.bitclk_sel = 3,
	.bitclk_div = 1,
	.pxclk_sel = 2,
	.pxclk_div = 6,
};

int lcd_lt8911ext_edp_1080p_init(void)
{
	int ret;

	ret = lcd_mipi_register_panel(&lcd_lt8911ext_edp_1080p);
	return ret;
}
