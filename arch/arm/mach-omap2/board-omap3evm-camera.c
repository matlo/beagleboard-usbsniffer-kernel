/*
 * arch/arm/mach-omap2/board-omap3evm-dc-v4l.c
 *
 * Driver for OMAP3 EVM Mass Market Daughter Card
 *
 * Copyright (C) 2008 Texas Instruments Inc
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * Contributors:
 *     Anuj Aggarwal <anuj.aggarwal@ti.com>
 *     Sivaraj R <sivaraj@ti.com>
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

#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/mm.h>
#include <linux/videodev2.h>
#include <linux/i2c/twl.h>
#include <linux/delay.h>

#include <plat/mux.h>
#include <plat/board.h>

#include <media/v4l2-int-device.h>
#include <media/tvp514x-int.h>

/* Include V4L2 ISP-Camera driver related header file */
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>

#include "mux.h"
#include "board-omap3evm-camera.h"

#define MODULE_NAME			"omap3evmdc"

#define TVP5146_I2C_BUSNUM		3
/* Is decoder present on-board or Daughter card */
static bool is_dec_onboard;

/* GPIO pins Daughter Card */
#define GPIO134_SEL_TVP_Y	(134)
#define GPIO54_SEL_EXP_CAM	(54)
#define GPIO136_SEL_CAM		(136)
/* GPIO pins for GEN_2 EVM */
#define GPIO98_VID_DEC_RES	(98)
#define nCAM_VD_SEL		(157)

#ifndef TRUE
#define TRUE			1
#endif
#ifndef FALSE
#define FALSE			0
#endif

#if defined(CONFIG_VIDEO_TVP514X) || defined(CONFIG_VIDEO_TVP514X_MODULE)
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
static struct omap34xxcam_hw_config decoder_hwc = {
	.dev_index		= 0,
	.dev_minor		= 0,
	.dev_type		= OMAP34XXCAM_SLAVE_SENSOR,
	.u.sensor.sensor_isp	= 1,
	.u.sensor.capture_mem	= PAGE_ALIGN(720*525*2*4),
};

static struct isp_interface_config tvp5146_if_config = {
	.ccdc_par_ser		= ISP_PARLL_YUV_BT,
	.dataline_shift		= 0x1,
	.hsvs_syncdetect	= ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe			= 0x0,
	.prestrobe		= 0x0,
	.shutter		= 0x0,
	.wait_hs_vs		= 2,
	.u.par.par_bridge	= 0x0,
	.u.par.par_clk_pol	= 0x0,
};
#endif

static struct v4l2_ifparm ifparm = {
	.if_type = V4L2_IF_TYPE_BT656,
	.u 	 = {
		.bt656 = {
			.frame_start_on_rising_vs = 1,
			.bt_sync_correct = 0,
			.swap		= 0,
			.latch_clk_inv	= 0,
			.nobt_hs_inv	= 0,	/* active high */
			.nobt_vs_inv	= 0,	/* active high */
			.mode		= V4L2_IF_TYPE_BT656_MODE_BT_8BIT,
			.clock_min	= TVP514X_XCLK_BT656,
			.clock_max	= TVP514X_XCLK_BT656,
		},
	},
};

/**
 * @brief tvp5146_ifparm - Returns the TVP5146 decoder interface parameters
 *
 * @param p - pointer to v4l2_ifparm structure
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_ifparm(struct v4l2_ifparm *p)
{
	if (p == NULL)
		return -EINVAL;

	*p = ifparm;
	return 0;
}

/**
 * @brief tvp5146_set_prv_data - Returns tvp5146 omap34xx driver private data
 *
 * @param priv - pointer to omap34xxcam_hw_config structure
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_set_prv_data(struct v4l2_int_device *s, void *priv)
{
#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
	struct omap34xxcam_hw_config *hwc = priv;

	if (priv == NULL)
		return -EINVAL;

	hwc->u.sensor.sensor_isp = decoder_hwc.u.sensor.sensor_isp;
	hwc->u.sensor.capture_mem = decoder_hwc.u.sensor.capture_mem;
	hwc->dev_index = decoder_hwc.dev_index;
	hwc->dev_minor = decoder_hwc.dev_minor;
	hwc->dev_type = decoder_hwc.dev_type;
	return 0;
#else
	return -EINVAL;
#endif
}

/**
 * @brief omap3evmdc_set_mux - Sets mux to enable/disable signal routing to
 *                             different peripherals present in board
 * IMPORTANT - This function will take care of writing appropriate values for
 * active low signals as well
 *
 * @param mux_id - enum, mux id to enable/disable
 * @param value - enum, ENABLE_MUX for enabling and DISABLE_MUX for disabling
 *
 * @return result of operation - 0 is success
 */
static int omap3evmdc_set_mux(enum omap3evmdc_mux mux_id, enum config_mux value)
{
	int err = 0;

	if (unlikely(mux_id >= NUM_MUX)) {
		printk(KERN_ERR MODULE_NAME ": Invalid mux id\n");
		return -EPERM;
	}

	switch (mux_id) {
	case MUX_TVP5146:
		if (ENABLE_MUX == value) {
			/* Enable TVP5146 Video in (GPIO134 = 0) */
			gpio_set_value(GPIO134_SEL_TVP_Y, 0);
			/* Disable Expansion Camera Video in (GPIO54 = 1) */
			gpio_set_value(GPIO54_SEL_EXP_CAM, 1);
			/* Disable Camera Video in (GPIO136 = 1)*/
			gpio_set_value(GPIO136_SEL_CAM, 1);
		} else {
			/* Disable TVP5146 Video in (GPIO134 = 0) */
			gpio_set_value(GPIO134_SEL_TVP_Y, 1);
		}
		break;

	case MUX_CAMERA_SENSOR:
		if (ENABLE_MUX == value) {
			/* Disable TVP5146 Video in (GPIO134 = 0) */
			gpio_set_value(GPIO134_SEL_TVP_Y, 1);
			/* Disable Exapansion Camera Video in (GPIO54 = 1) */
			gpio_set_value(GPIO54_SEL_EXP_CAM, 1);
			/* Enable Camera Video in (GPIO136 = 1) */
			gpio_set_value(GPIO136_SEL_CAM, 0);
		} else {
			/* Disable Camera Video in (GPIO136 = 1) */
			gpio_set_value(GPIO136_SEL_CAM, 1);
		}
		break;

	case MUX_EXP_CAMERA_SENSOR:
		if (ENABLE_MUX == value) {
			/* Disable TVP5146 Video in (GPIO134 = 0) */
			gpio_set_value(GPIO134_SEL_TVP_Y, 1);
			/* Enable Expansion Camera Video in (GPIO54 = 1) */
			gpio_set_value(GPIO54_SEL_EXP_CAM, 0);
			/* Disable Camera Video in (GPIO136 = 1) */
			gpio_set_value(GPIO136_SEL_CAM, 1);
		} else {
			/* Disable Expansion Camera Video in (GPIO54 = 1) */
			gpio_set_value(GPIO54_SEL_EXP_CAM, 1);
		}
		break;

	case NUM_MUX:
	default:
		printk(KERN_ERR "Invalid mux id\n");
		err = -EPERM;
	}

	return err;
}

/**
 * @brief omap3evm_set_mux - Sets mux to enable/disable signal routing to
 *                             different peripherals present on new EVM board
 * IMPORTANT - This function will take care of writing appropriate values for
 * active low signals as well
 *
 * @param mux_id - enum, mux id to enable/disable
 * @param value - enum, ENABLE_MUX for enabling and DISABLE_MUX for disabling
 *
 * @return result of operation - 0 is success
 */
static int omap3evm_set_mux(enum omap3evmdc_mux mux_id, enum config_mux value)
{
	static int is_init_done = 0;
	unsigned char val;
	int err = 0;

	if (unlikely(mux_id >= NUM_MUX)) {
		printk(KERN_ERR MODULE_NAME ": Invalid mux id\n");
		return -EPERM;
	}

	if (is_init_done == 0) {
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
		twl_i2c_write_u8(TWL4030_MODULE_GPIO, val, REG_GPIOPUPDCTR3);

		/* Assert the reset signal */
		gpio_set_value(GPIO98_VID_DEC_RES, 0);
		mdelay(5);
		gpio_set_value(GPIO98_VID_DEC_RES, 1);
	}
	switch (mux_id) {
	case MUX_TVP5146:
		if (ENABLE_MUX == value) {
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
		if (ENABLE_MUX == value) {
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
		if (ENABLE_MUX == value) {
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

	case NUM_MUX:
	default:
		printk(KERN_ERR "Invalid mux id\n");
		err = -EPERM;
	}

	return err;
}
/**
 * @brief tvp5146_power_set - Power-on or power-off TVP5146 device
 *
 * @param power - enum, Power on/off, resume/standby
 *
 * @return result of operation - 0 is success
 */
static int tvp5146_power_set(struct v4l2_int_device *s, enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;

	switch (power) {
	case V4L2_POWER_OFF:
		/* Disable mux for TVP5146 decoder data path */
		if (is_dec_onboard) {
			if (omap3evm_set_mux(MUX_TVP5146, DISABLE_MUX))
				return -ENODEV;
		} else {
			if (omap3evmdc_set_mux(MUX_TVP5146, DISABLE_MUX))
				return -ENODEV;
		}
		break;

	case V4L2_POWER_STANDBY:
		break;

	case V4L2_POWER_ON:
		/* Enable mux for TVP5146 decoder data path */
		if (is_dec_onboard) {
			if (omap3evm_set_mux(MUX_TVP5146, ENABLE_MUX))
				return -ENODEV;
		} else {
			if (omap3evmdc_set_mux(MUX_TVP5146, ENABLE_MUX))
				return -ENODEV;
		}

#if defined(CONFIG_VIDEO_OMAP3) || defined(CONFIG_VIDEO_OMAP3_MODULE)
		isp_configure_interface(vdev->cam->isp, &tvp5146_if_config);
#endif
		break;

	default:
		return -ENODEV;
		break;
	}
	return 0;
}

static struct tvp514x_platform_data tvp5146_pdata = {
	.master		= "omap34xxcam",
	.power_set	= tvp5146_power_set,
	.priv_data_set	= tvp5146_set_prv_data,
	.ifparm		= tvp5146_ifparm,
	/* Some interface dependent params */
	.clk_polarity	= 0, /* data clocked out on falling edge */
	.hs_polarity	= 1, /* 0 - Active low, 1- Active high */
	.vs_polarity	= 1, /* 0 - Active low, 1- Active high */
};

static struct i2c_board_info __initdata tvp5146_i2c_board_info = {
	I2C_BOARD_INFO("tvp5146m2", 0),
	.platform_data	= &tvp5146_pdata,
};

#endif				/* #ifdef CONFIG_VIDEO_TVP514X */

/**
 * @brief omap3evmdc_mdc_config - GPIO configuration for
 *                          GPIO 134, 54 and 136
 *
 * @return result of operation - 0 is success
 */
static int omap3evmdc_mdc_config(void)
{
	if (is_dec_onboard) {
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
	} else {

		/* Setting the MUX configuration */
		omap_mux_init_gpio(134, OMAP_PIN_INPUT_PULLUP);
		omap_mux_init_gpio(54, OMAP_PIN_INPUT_PULLUP);
		omap_mux_init_gpio(136, OMAP_PIN_INPUT_PULLUP);

		if (gpio_request(GPIO134_SEL_TVP_Y, "TVP5146 Vid-in") < 0) {
			printk(KERN_ERR MODULE_NAME ": Can't get GPIO 134\n");
			return -EINVAL;
		}

		if (gpio_request(GPIO54_SEL_EXP_CAM, "EXP_CAM Vid-in") < 0) {
			printk(KERN_ERR MODULE_NAME ": Can't get GPIO 54\n");
			return -EINVAL;
		}

		if (gpio_request(GPIO136_SEL_CAM, "CAM Vid-in") < 0) {
			printk(KERN_ERR MODULE_NAME ": Can't get GPIO 136\n");
			return -EINVAL;
		}

		/* Make GPIO as output */
		gpio_direction_output(GPIO134_SEL_TVP_Y, 0);
		gpio_direction_output(GPIO54_SEL_EXP_CAM, 0);
		gpio_direction_output(GPIO136_SEL_CAM, 0);
	}

	return 0;
}

/**
 * @brief omap3evmdc_init - module init function. Should be called before any
 *                          client driver init call
 *
 * @return result of operation - 0 is success
 */
int __init omap3evmdc_init(void)
{
	int err;

	/* Status of Video Decoder : On Board or DC */
	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2)
		is_dec_onboard = TRUE;
	else
		is_dec_onboard = FALSE;

	err = omap3evmdc_mdc_config();
	if (err) {
		printk(KERN_ERR MODULE_NAME ": MDC configuration failed \n");
		return err;
	}

	/*
	 * Register the I2C devices present in the board to the I2C
	 * framework.
	 * If more I2C devices are added, then each device information should
	 * be registered with I2C using i2c_register_board_info().
	 */
#if defined(CONFIG_VIDEO_TVP514X) || defined(CONFIG_VIDEO_TVP514X_MODULE)
	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2)
		tvp5146_i2c_board_info.addr = 0x5C;
	else
		tvp5146_i2c_board_info.addr = 0x5D;

	err = i2c_register_board_info(TVP5146_I2C_BUSNUM,
					&tvp5146_i2c_board_info, 1);
	if (err) {
		printk(KERN_ERR MODULE_NAME \
				": TVP5146 I2C Board Registration failed \n");
		return err;
	}
#endif
	printk(KERN_INFO MODULE_NAME ": Driver registration complete \n");

	return 0;
}
arch_initcall(omap3evmdc_init);
