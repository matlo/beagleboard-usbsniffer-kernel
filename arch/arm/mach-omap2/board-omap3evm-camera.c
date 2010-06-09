/*
 * arch/arm/mach-omap2/board-omap3evm-camera.c
 *
 * Driver for OMAP3 EVM Video Decoder
 *
 * Copyright (C) 2010 Texas Instruments Inc
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/i2c.h>
#include <linux/i2c/twl.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <linux/string.h>

#include <media/v4l2-subdev.h>
#include <media/tvp514x.h>

#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/isp.h>
#include "mux.h"

extern struct platform_device omap3isp_device;

/* macro definations */
#define VIDEO_DEC_SUB_DEVICE	"tvp5146m2"

/* GPIO pins Daughter Card */
#define GPIO134_SEL_TVP_Y       (134)
#define GPIO54_SEL_EXP_CAM      (54)
#define GPIO136_SEL_CAM         (136)
/* GPIO pins for GEN_2 EVM */
#define GPIO98_VID_DEC_RES      (98)
#define nCAM_VD_SEL             (157)

/* mux id to enable/disable signal routing to different peripherals */
enum omap3evmdc_mux {
	MUX_TVP5146 = 0,
	MUX_CAMERA_SENSOR,
	MUX_EXP_CAMERA_SENSOR,
	MUX_INVAL,
};

/**
 * Sets mux to enable/disable signal routing to different
 * peripherals (decoder/sensor/Ext Sensor) present on new EVM board
 *
 */
static int omap3evm_set_mux(enum omap3evmdc_mux mux_id, int on)
{
	unsigned char val;
	int err = 0;

	if (unlikely(mux_id >= MUX_INVAL)) {
		printk(KERN_ERR ": Invalid mux id\n");
		return -EINVAL;
	}

	/*FIXME: Need to follow standard GPIO API's to control
	 *	TWL4030 GPIO.
	 */
	/* Enable TWL GPIO Module */
	twl_i2c_write_u8(TWL4030_MODULE_GPIO, 0x04, REG_GPIO_CTRL);

	/* First Level Enable GPIO */
	/* Configure GPIO2 as output */
	twl_i2c_read_u8(TWL4030_MODULE_GPIO, &val, REG_GPIODATADIR1);
	val |= 0x04;
	twl_i2c_write_u8(TWL4030_MODULE_GPIO, val, REG_GPIODATADIR1);
	/* Set GPIO-2 pull-up */
	twl_i2c_read_u8(TWL4030_MODULE_GPIO, &val, REG_GPIOPUPDCTR1);
	val |= 0x20;
	twl_i2c_write_u8(TWL4030_MODULE_GPIO, val, REG_GPIOPUPDCTR1);
	/* Set GPIO-2 = 0 */
	twl_i2c_read_u8(TWL4030_MODULE_GPIO, &val, REG_GPIODATAOUT1);
	val &= ~0x04;
	twl_i2c_write_u8(TWL4030_MODULE_GPIO, val, REG_GPIODATAOUT1);

	/* Configure GPIO8 as output*/
	twl_i2c_read_u8(TWL4030_MODULE_GPIO, &val, REG_GPIODATADIR2);
	val |= 0x1;
	twl_i2c_write_u8(TWL4030_MODULE_GPIO, val, REG_GPIODATADIR2);
	/* GPIO-8 pull down */
	twl_i2c_read_u8(TWL4030_MODULE_GPIO, &val, REG_GPIOPUPDCTR3);
	val |= 0x01;
	twl_i2c_write_u8(TWL4030_MODULE_GPIO, val, REG_GPIODATADIR2);
	/* GPIO-8 pull down */
	twl_i2c_read_u8(TWL4030_MODULE_GPIO, &val, REG_GPIOPUPDCTR3);
	val |= 0x01;
	twl_i2c_write_u8(TWL4030_MODULE_GPIO, val, REG_GPIOPUPDCTR3);

	/* Assert the reset signal */
	gpio_set_value(GPIO98_VID_DEC_RES, 0);
	mdelay(5);
	gpio_set_value(GPIO98_VID_DEC_RES, 1);

	switch (mux_id) {
	case MUX_TVP5146:
		if (on) {
			/* Set GPIO8 = 0 */
			twl_i2c_read_u8(TWL4030_MODULE_GPIO, &val,
					REG_GPIODATAOUT2);
			val &= ~0x1;
			twl_i2c_write_u8(TWL4030_MODULE_GPIO, val,
					REG_GPIODATAOUT2);

			gpio_set_value(nCAM_VD_SEL, 1);
		} else {
			/* Set GPIO8 = 0 */
			twl_i2c_read_u8(TWL4030_MODULE_GPIO, &val,
					REG_GPIODATAOUT2);
			val |= 0x1;
			twl_i2c_write_u8(TWL4030_MODULE_GPIO, val,
					REG_GPIODATAOUT2);
		}
		break;

	case MUX_CAMERA_SENSOR:
		if (on) {
			/* Set GPIO8 = 0 */
			twl_i2c_read_u8(TWL4030_MODULE_GPIO, &val,
					REG_GPIODATAOUT2);
			val &= ~0x1;
			twl_i2c_write_u8(TWL4030_MODULE_GPIO, val,
					REG_GPIODATAOUT2);

			gpio_set_value(nCAM_VD_SEL, 0);
		} else {
			/* Set GPIO8 = 0 */
			twl_i2c_read_u8(TWL4030_MODULE_GPIO, &val,
					REG_GPIODATAOUT2);
			val |= 0x1;
			twl_i2c_write_u8(TWL4030_MODULE_GPIO, val,
					REG_GPIODATAOUT2);
		}
		break;

	case MUX_EXP_CAMERA_SENSOR:
		if (on) {
			/* Set GPIO8 = 1 */
			twl_i2c_read_u8(TWL4030_MODULE_GPIO, &val,
					REG_GPIODATAOUT2);
			val |= 0x1;
			twl_i2c_write_u8(TWL4030_MODULE_GPIO, val,
					REG_GPIODATAOUT2);

		} else {
			/* Set GPIO8 = 0 */
			twl_i2c_read_u8(TWL4030_MODULE_GPIO, &val,
					REG_GPIODATAOUT2);
			val &= ~0x1;
			twl_i2c_write_u8(TWL4030_MODULE_GPIO, val,
					REG_GPIODATAOUT2);
		}
		break;

	case MUX_INVAL:
	default:
		printk(KERN_ERR "Invalid mux id\n");
		err = -EINVAL;
	}

	return err;
}

static int __init omap3evm_camera_hw_init(void)
{
	/* Enable Video Decoder */
	omap_mux_init_gpio(157, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(nCAM_VD_SEL, "Vid-Dec Sel") < 0) {
		printk(KERN_ERR "Failed to get GPIO 157\n");
		return -EINVAL;
	}
	gpio_direction_output(nCAM_VD_SEL, 1);
	gpio_set_value(nCAM_VD_SEL, 1);

	omap_mux_init_gpio(98, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(GPIO98_VID_DEC_RES, "vid-dec reset") < 0) {
		printk(KERN_ERR "failed to get GPIO98_VID_DEC_RES\n");
		return -EINVAL;
	}
	gpio_direction_output(GPIO98_VID_DEC_RES, 1);

	return 0;
}


/*
 * VPFE - Video Decoder interface
 */
#if 0
#define TVP514X_STD_ALL		(V4L2_STD_NTSC | V4L2_STD_PAL)

/* Inputs available at the TVP5146 */
static struct v4l2_input tvp5146_inputs[] = {
	{
		.index  = 0,
		.name   = "Composite",
		.type   = V4L2_INPUT_TYPE_CAMERA,
		.std    = TVP514X_STD_ALL,
	},
	{
		.index  = 1,
		.name   = "S-Video",
		.type   = V4L2_INPUT_TYPE_CAMERA,
		.std    = TVP514X_STD_ALL,
	},
};
#endif

static struct isp_platform_data omap3evm_isp_platform_data = {
	.parallel = {
		.data_lane_shift = 1,
		.bridge = 0,
		.clk_pol = 0,
	},
};


static int tvp5146_power_set(struct v4l2_subdev *subdev, int on)
{
	/*
	 * TODO: Add changes for Sensor interface
	 */

	/* Enable mux for TVP5146 decoder data path */
	if (omap3evm_set_mux(MUX_TVP5146, on))
		return -ENODEV;

	return 0;
}

static struct tvp514x_platform_data tvp514x_platform_data = {
	.set_power		= tvp5146_power_set,
	.clk_polarity		= 0,
	.hs_polarity		= 1,
	.vs_polarity		= 1,
};


static struct i2c_board_info omap3evm_camera_i2c_device = {
	I2C_BOARD_INFO(VIDEO_DEC_SUB_DEVICE, 0x5C),
	.type		= VIDEO_DEC_SUB_DEVICE,
	.platform_data	= &tvp514x_platform_data,
};

static struct v4l2_subdev_i2c_board_info omap3evm_camera_primary_subdevs[] = {
	{
		.board_info	= &omap3evm_camera_i2c_device,
		.i2c_adapter_id	= 3,
		.module_name	= VIDEO_DEC_SUB_DEVICE,

	},
	{ NULL, 0, NULL, },
};

static struct omap34xxcam_platform_data omap3evm_camera_pdata = {
	.isp		= &omap3isp_device,
	.subdevs[0]	= omap3evm_camera_primary_subdevs,
	.sensors[0]	= {
		.interface	= ISP_INTERFACE_PARALLEL,
		.capture_mem	= PAGE_ALIGN(2608 * 1966 * 2) * 2,
		.ival_default	= { 1, 30 },
	},
};

static void omap3evm_camera_release(struct device *dev)
{
}

static struct platform_device omap3evm_camera_device = {
	.name	= "omap34xxcam",
	.id	= 0,
	.dev	= {
		.platform_data	= &omap3evm_camera_pdata,
		.release	= omap3evm_camera_release,
	},
};

static int __init omap3evm_camera_init(void)
{
	int err;

	err = omap3evm_camera_hw_init();
	if (err)
		return err;

	omap3isp_device.dev.platform_data = &omap3evm_isp_platform_data;

	return platform_device_register(&omap3evm_camera_device);
}

static void __exit omap3evm_camera_exit(void)
{
}

module_init(omap3evm_camera_init);
module_exit(omap3evm_camera_exit);

MODULE_LICENSE("GPL");
