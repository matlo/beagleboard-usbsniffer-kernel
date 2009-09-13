/*
 * linux/arch/arm/mach-omap2/board-ldp-camera.c
 *
 * Copyright (C) 2009 Texas Instruments Inc.
 * Sergio Aguirre <saaguirre@ti.com>
 *
 * Modified from mach-omap2/board-ldp.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifdef CONFIG_TWL4030_CORE

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mm.h>

#include <linux/i2c/twl4030.h>

#include <asm/io.h>

#include <mach/gpio.h>

static int cam_inited;
#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>

#define LDPCAM_USE_XCLKB	1

#define VAUX_1_8_V		0x05
#define VAUX_DEV_GRP_P1		0x20
#define VAUX_DEV_GRP_NONE	0x00

#if defined(CONFIG_VIDEO_OV3640) || defined(CONFIG_VIDEO_OV3640_MODULE)
#define OV3640_RESET_GPIO  	98
#define OV3640_STANDBY_GPIO	7
#include <media/ov3640.h>
#include <../drivers/media/video/isp/ispcsi2.h>
#define OV3640_CSI2_CLOCK_POLARITY	0	/* +/- pin order */
#define OV3640_CSI2_DATA0_POLARITY	0	/* +/- pin order */
#define OV3640_CSI2_DATA1_POLARITY	0	/* +/- pin order */
#define OV3640_CSI2_CLOCK_LANE		1	 /* Clock lane position: 1 */
#define OV3640_CSI2_DATA0_LANE		2	 /* Data0 lane position: 2 */
#define OV3640_CSI2_DATA1_LANE		3	 /* Data1 lane position: 3 */
#define OV3640_CSI2_PHY_THS_TERM	4
#define OV3640_CSI2_PHY_THS_SETTLE	14
#define OV3640_CSI2_PHY_TCLK_TERM	0
#define OV3640_CSI2_PHY_TCLK_MISS	1
#define OV3640_CSI2_PHY_TCLK_SETTLE	14

#define OV3640_BIGGEST_FRAME_BYTE_SIZE	PAGE_ALIGN(2048 * 1536 * 2)

static struct omap34xxcam_sensor_config ov3640_hwc = {
	.sensor_isp = 0,
	.capture_mem = OV3640_BIGGEST_FRAME_BYTE_SIZE * 2,
	.ival_default	= { 1, 15 },
};

static struct isp_interface_config ov3640_if_config = {
	.ccdc_par_ser = ISP_CSIA,
	.dataline_shift = 0x0,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.prev_sph = 2,
	.prev_slv = 1,
	.wenlog = ISPCCDC_CFG_WENLOG_AND,
	.wait_hs_vs = 2,
	.u.csi.crc = 0x0,
	.u.csi.mode = 0x0,
	.u.csi.edge = 0x0,
	.u.csi.signalling = 0x0,
	.u.csi.strobe_clock_inv = 0x0,
	.u.csi.vs_edge = 0x0,
	.u.csi.channel = 0x1,
	.u.csi.vpclk = 0x1,
	.u.csi.data_start = 0x0,
	.u.csi.data_size = 0x0,
	.u.csi.format = V4L2_PIX_FMT_SGRBG10,
};

static int ov3640_sensor_set_prv_data(struct v4l2_int_device *s, void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.sensor_isp = ov3640_hwc.sensor_isp;
	hwc->dev_index = 1;
	hwc->dev_minor = 4;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	return 0;
}

static int ov3640_sensor_power_set(struct v4l2_int_device *s,
				   enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct isp_csi2_lanes_cfg lanecfg;
	struct isp_csi2_phy_cfg phyconfig;
	static enum v4l2_power previous_power = V4L2_POWER_OFF;

	if (!cam_inited) {
		printk(KERN_ERR "OV3640: Unable to control board GPIOs!\n");
		return -EFAULT;
	}

	switch (power) {
	case V4L2_POWER_ON:
		if (previous_power == V4L2_POWER_OFF)
			isp_csi2_reset();
		lanecfg.clk.pol = OV3640_CSI2_CLOCK_POLARITY;
		lanecfg.clk.pos = OV3640_CSI2_CLOCK_LANE;
		lanecfg.data[0].pol = OV3640_CSI2_DATA0_POLARITY;
		lanecfg.data[0].pos = OV3640_CSI2_DATA0_LANE;
		lanecfg.data[1].pol = OV3640_CSI2_DATA1_POLARITY;
		lanecfg.data[1].pos = OV3640_CSI2_DATA1_LANE;
		lanecfg.data[2].pol = 0;
		lanecfg.data[2].pos = 0;
		lanecfg.data[3].pol = 0;
		lanecfg.data[3].pos = 0;
		isp_csi2_complexio_lanes_config(&lanecfg);
		isp_csi2_complexio_lanes_update(true);

		phyconfig.ths_term = OV3640_CSI2_PHY_THS_TERM;
		phyconfig.ths_settle = OV3640_CSI2_PHY_THS_SETTLE;
		phyconfig.tclk_term = OV3640_CSI2_PHY_TCLK_TERM;
		phyconfig.tclk_miss = OV3640_CSI2_PHY_TCLK_MISS;
		phyconfig.tclk_settle = OV3640_CSI2_PHY_TCLK_SETTLE;
		isp_csi2_phy_config(&phyconfig);
		isp_csi2_phy_update(true);

		isp_configure_interface(vdev->cam->isp, &ov3640_if_config);

		if (previous_power == V4L2_POWER_OFF) {
			/* turn on analog power */
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_1_8_V, TWL4030_VAUX4_DEDICATED);
			twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
					VAUX_DEV_GRP_P1, TWL4030_VAUX4_DEV_GRP);
			udelay(100);
			/* Turn ON Omnivision sensor */
			gpio_set_value(OV3640_RESET_GPIO, 1);
			gpio_set_value(OV3640_STANDBY_GPIO, 0);
			udelay(100);

			/* RESET Omnivision sensor */
			gpio_set_value(OV3640_RESET_GPIO, 0);
			udelay(100);
			gpio_set_value(OV3640_RESET_GPIO, 1);
		}
		break;
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		isp_csi2_complexio_power(ISP_CSI2_POWER_OFF);
		twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER,
				VAUX_DEV_GRP_NONE, TWL4030_VAUX4_DEV_GRP);
		break;
	case V4L2_POWER_STANDBY:
		break;
	}
	previous_power = power;
	return 0;
}

static u32 ov3640_sensor_set_xclk(struct v4l2_int_device *s, u32 xclkfreq)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;

	return isp_set_xclk(vdev->cam->isp, xclkfreq, LDPCAM_USE_XCLKB);
}

struct ov3640_platform_data ldp_ov3640_platform_data = {
	.power_set	 = ov3640_sensor_power_set,
	.priv_data_set	 = ov3640_sensor_set_prv_data,
	.set_xclk	 = ov3640_sensor_set_xclk,
};

#endif

void __init ldp_cam_init(void)
{
#if defined(CONFIG_VIDEO_OV3640) || defined(CONFIG_VIDEO_OV3640_MODULE)
	cam_inited = 0;
	/* Request and configure gpio pins */
	if (gpio_request(OV3640_RESET_GPIO, "ov3640_reset_gpio") != 0) {
		printk(KERN_ERR "Could not request GPIO %d",
					OV3640_RESET_GPIO);
		return;
	}
	if (gpio_request(OV3640_STANDBY_GPIO, "ov3640_standby_gpio") != 0) {
		printk(KERN_ERR "Could not request GPIO %d",
					OV3640_STANDBY_GPIO);
		gpio_free(OV3640_RESET_GPIO);
		return;
	}
	/* set to output mode */
	gpio_direction_output(OV3640_RESET_GPIO, true);
	gpio_direction_output(OV3640_STANDBY_GPIO, true);
#endif
	cam_inited = 1;
}
#else
void __init ldp_cam_init(void)
{
}
#endif
