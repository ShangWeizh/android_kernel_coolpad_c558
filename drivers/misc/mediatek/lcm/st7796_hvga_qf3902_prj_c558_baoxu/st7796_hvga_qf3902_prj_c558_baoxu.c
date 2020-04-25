/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include "lcm_drv.h"
#include "mtk_auxadc.h"
#include <linux/string.h>

#define REGFLAG_DELAY 0xFE
#define REGFLAG_END_OF_TABLE 0xFF

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v) lcm_util.set_reset_pin(v)

#define UDELAY(n) lcm_util.udelay(n)
#define MDELAY(n) lcm_util.mdelay(n)

#define dsi_set_cmdq_V2(cmd, count, para_list, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, para_list, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define dsi_write_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define dsi_write_regs(addr, para, nums) lcm_util.dsi_write_regs(addr, para, nums)
#define dsi_dcs_read_lcm_reg(cmd) lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size) lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[32];
};

#define MIN_ID_VOLTAGE 1400

static struct LCM_setting_table lcm_initialization_setting[] = {
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 5, {}},
	{0xF0, 1, {0xC3}},
	{0xF0, 1, {0x96}},
	{0x36, 1, {0x48}},
	{0x3A, 1, {0x77}},
	{0xE8, 8, {0x40, 0x82, 0x07, 0x18, 0x27, 0x0A, 0xB6, 0x33}},
	{0xB9, 1, {0x33}},
	{0xC0, 2, {0x80, 0x25}},
	{0xC2, 1, {0xA7}},
	{0xC5, 1, {0x27}},
	{0xE0, 14, {0xF0, 0x01, 0x03, 0x07, 0x09, 0x2C, 0x3B, 0x44, 0x4A, 0x16, 0x0D, 0x0D, 0x1D, 0x20}},
	{0xE1, 14, {0xF0, 0x00, 0x03, 0x08, 0x0A, 0x26, 0x37, 0x44, 0x4C, 0x0D, 0x1C, 0x1B, 0x1D, 0x20}},
	{0xF0, 1, {0x3C}},
	{0xF0, 1, {0x69}},
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = {
	// Sleep Out
	{0x11, 0, {0x00}},
	{REGFLAG_DELAY, 5, {}},

	// Display On
	{0x29, 0, {0x00}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display Off
	{0x28, 0, {0x00}},

	// Sleep In
	{0x10, 0, {0x00}},
	{REGFLAG_DELAY, 5, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;
	for (i = 0; i < count; i++) {
		struct LCM_setting_table t = table[i];

		switch (t.cmd) {
		case REGFLAG_DELAY:
			MDELAY(t.count);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			dsi_set_cmdq_V2(t.cmd, t.count, t.para_list, force_update);
			break;
		}
	}
}

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->dbi.te_mode = LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity = LCM_POLARITY_RISING;
	params->dsi.mode = BURST_VDO_MODE;
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->type = LCM_TYPE_DSI;
	params->dsi.packet_size = 256;
	params->width = 320;
	params->dsi.vertical_sync_active = 4;
	params->height = 480;
	params->dsi.vertical_backporch = 16;
	params->dsi.LANE_NUM = LCM_ONE_LANE;
	params->dsi.vertical_frontporch = 20;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;
	params->dsi.horizontal_sync_active = 10;
	params->dsi.intermediat_buffer_num = 2;
	params->dsi.horizontal_backporch = 80;
	params->dsi.horizontal_frontporch = 80;
	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;
	params->dsi.vertical_active_line = 480;
	params->dsi.horizontal_active_pixel = 320;
	params->dsi.PLL_CLOCK = 160;
}

static void lcm_init(void)
{
	SET_RESET_PIN(0);
	MDELAY(120);
	SET_RESET_PIN(1);
	MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_resume(void)
{
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
}

static unsigned int lcm_compare_id(void)
{

	return 1;

#if 0
	int res;
	int data[4] = {0, 0, 0, 0};
	int rawdata = 0;
	int lcm_voltage;

	SET_RESET_PIN(0);
	MDELAY(120);
	SET_RESET_PIN(1);
	MDELAY(120);

	res = IMM_GetOneChannelValue(0x0C, data, &rawdata);
	if (res < 0) {
		// Failed to get the lcm_voltage reading
		return 0;
	}

	lcm_voltage = data[0] * 1000 + data[1] * 10;

	if (lcm_voltage > MIN_ID_VOLTAGE) {
		return 1;
	}

	return 0;
#endif
}

LCM_DRIVER st7796_hvga_qf3902_prj_c558_baoxu_lcm_drv =
{
	.name = "st7796_hvga_qf3902_prj_c558_baoxu",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id = lcm_compare_id,
};
