/*
 * Copyright (C) 2009 Texas Instruments Inc.
 * Mikkel Christensen <mlc@ti.com>
 *
 * Modified from mach-omap2/board-ldp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/regulator/machine.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/common.h>
#include <plat/board.h>

#include <mach/board-zoom.h>

#include "mux.h"
#include "sdram-micron-mt46h32m32lf-6.h"
#include "omap3-opp.h"

#include <media/v4l2-int-device.h>

#if defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)
#include <media/imx046.h>
extern struct imx046_platform_data zoom2_imx046_platform_data;
#endif

extern void zoom2_cam_init(void);

#ifdef CONFIG_VIDEO_LV8093
#include <media/lv8093.h>
extern struct imx046_platform_data zoom2_lv8093_platform_data;
#endif

static struct platform_device zoom2_cam_device = {
	.name		= "zoom2_cam",
	.id		= -1,
};

static struct regulator_consumer_supply zoom2_vaux2_supplies[] = {
	{
		.supply		= "vaux2_1",
		.dev		= &zoom2_cam_device.dev,
	},
};

static struct regulator_consumer_supply zoom2_vaux4_supplies[] = {
	{
		.supply		= "vaux4_1",
		.dev		= &zoom2_cam_device.dev,
	},
};

static struct regulator_init_data zoom2_vaux2 = {
	.constraints = {
		.min_uV			= 2800000,
		.max_uV			= 2800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(zoom2_vaux2_supplies),
	.consumer_supplies	= zoom2_vaux2_supplies,
};

/* VAUX4 for OMAP VDD_CSI2 (camera) */
static struct regulator_init_data zoom2_vaux4 = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(zoom2_vaux4_supplies),
	.consumer_supplies	= zoom2_vaux4_supplies,
};

static void __init omap_zoom2_init_irq(void)
{
	omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
				 mt46h32m32lf6_sdrc_params, omap3_mpu_rate_table,
				 omap3_dsp_rate_table, omap3_l3_rate_table);
	omap_init_irq();
	omap_gpio_init();
}

/* REVISIT: These audio entries can be removed once MFD code is merged */
#if 0

static struct twl4030_madc_platform_data zoom2_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_codec_audio_data zoom2_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data zoom2_codec_data = {
	.audio_mclk = 26000000,
	.audio = &zoom2_audio_data,
};

static struct twl4030_platform_data zoom2_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.bci		= &zoom2_bci_data,
	.madc		= &zoom2_madc_data,
	.usb		= &zoom2_usb_data,
	.gpio		= &zoom2_gpio_data,
	.keypad		= &zoom2_kp_twl4030_data,
	.codec		= &zoom2_codec_data,
	.vaux2		= &zoom2_vaux2,
	.vaux4		= &zoom2_vaux4,
	.vmmc1          = &zoom2_vmmc1,
	.vmmc2          = &zoom2_vmmc2,
	.vsim           = &zoom2_vsim,

};

#endif

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct i2c_board_info __initdata zoom2_i2c_boardinfo2[] = {
#if defined(CONFIG_VIDEO_IMX046) || defined(CONFIG_VIDEO_IMX046_MODULE)
	{
		I2C_BOARD_INFO("imx046", IMX046_I2C_ADDR),
		.platform_data = &zoom2_imx046_platform_data,
	},
#endif
#ifdef CONFIG_VIDEO_LV8093
	{
		I2C_BOARD_INFO(LV8093_NAME,  LV8093_AF_I2C_ADDR),
		.platform_data = &zoom2_lv8093_platform_data,
	},
#endif
};

static int __init omap_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, NULL, 0);
	omap_register_i2c_bus(2, 100, zoom2_i2c_boardinfo2,
			ARRAY_SIZE(zoom2_i2c_boardinfo2));
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

extern int __init omap_zoom2_debugboard_init(void);

static struct platform_device *zoom2_devices[] __initdata = {
	&zoom2_cam_device,
};

static void __init omap_zoom2_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	zoom_peripherals_init();
	zoom_debugboard_init();

	omap_i2c_init();
	platform_add_devices(zoom2_devices, ARRAY_SIZE(zoom2_devices));
	zoom2_cam_init();
}

static void __init omap_zoom2_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP_ZOOM2, "OMAP Zoom2 board")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap_zoom2_map_io,
	.init_irq	= omap_zoom2_init_irq,
	.init_machine	= omap_zoom2_init,
	.timer		= &omap_timer,
MACHINE_END
