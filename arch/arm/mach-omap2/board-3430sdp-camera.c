/*
 * linux/arch/arm/mach-omap2/board-3430sdp.c
 *
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-generic.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mm.h>

#include <linux/regulator/consumer.h>

#include <asm/io.h>

#include <mach/mux.h>
#include <mach/gpio.h>
#ifdef CONFIG_OMAP_PM_SRF
#include <mach/omap-pm.h>
#endif

static int cam_inited;

static struct device *camkit_dev;

#include <media/v4l2-int-device.h>
#include <../drivers/media/video/omap34xxcam.h>
#include <../drivers/media/video/isp/ispreg.h>
#define DEBUG_BASE			0x08000000

#define REG_SDP3430_FPGA_GPIO_2		(0x50)
#define FPGA_SPR_GPIO1_3v3		(0x1 << 14)
#define FPGA_GPIO6_DIR_CTRL		(0x1 << 6)

#define CAMKITV3_USE_XCLKA		0
#define CAMKITV3_USE_XCLKB		1

#define CAMKITV3_RESET_GPIO		98

/* Sensor specific GPIO signals */
#define MT9P012_STANDBY_GPIO		58
#define OV3640_STANDBY_GPIO		55
#define TPS61059_TORCH_EN_GPIO		56
#define TPS61059_FLASH_STROBE_GPIO	126

static struct regulator *sdp3430_mt9p012_reg;
static struct regulator *sdp3430_dw9710_reg;
static struct regulator *sdp3430_ov3640_reg;

#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
#include <media/mt9p012.h>
static enum v4l2_power mt9p012_previous_power = V4L2_POWER_OFF;

#define MT9P012_BIGGEST_FRAME_BYTE_SIZE	PAGE_ALIGN(2592 * 1944 * 2)

#ifdef CONFIG_VIDEO_DW9710
#include <media/dw9710.h>
#define is_dw9710_enabled()		1
#else
#define is_dw9710_enabled()		0
#endif /* CONFIG_VIDEO_DW9710 */

#if defined(CONFIG_VIDEO_TPS61059) || defined(CONFIG_VIDEO_TPS61059_MODULE)
#include <media/tps61059.h>
#define is_tps61059_enabled()		1
#else
#define is_tps61059_enabled()		0
#endif /* CONFIG_VIDEO_TPS61059 || CONFIG_VIDEO_TPS61059_MODULE */

#define is_mt9p012_enabled()		1
#else /* CONFIG_VIDEO_MT9P012 || CONFIG_VIDEO_MT9P012_MODULE */
#define is_mt9p012_enabled()		0
#define is_dw9710_enabled()		0
#define is_tps61059_enabled()		0
#endif /* CONFIG_VIDEO_MT9P012 || CONFIG_VIDEO_MT9P012_MODULE */

#if defined(CONFIG_VIDEO_OV3640) || defined(CONFIG_VIDEO_OV3640_MODULE)
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
#define is_ov3640_enabled()		1
#else
#define is_ov3640_enabled()		0
#endif /* CONFIG_VIDEO_OV3640 || CONFIG_VIDEO_OV3640_MODULE */

#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE) || \
    defined(CONFIG_VIDEO_OV3640) || defined(CONFIG_VIDEO_OV3640_MODULE) || \
    defined(CONFIG_VIDEO_DW9710)

static void enable_fpga_vio_1v8(u8 enable)
{
	void __iomem *fpga_map_addr;
	u16 reg_val;

	fpga_map_addr = ioremap(DEBUG_BASE, 4096);
	reg_val = readw(fpga_map_addr + REG_SDP3430_FPGA_GPIO_2);

	/* Ensure that the SPR_GPIO1_3v3 is 0 - powered off.. 1 is on */
	if (reg_val & FPGA_SPR_GPIO1_3v3) {
		reg_val |= FPGA_SPR_GPIO1_3v3;
		reg_val |= FPGA_GPIO6_DIR_CTRL; /* output mode */
		writew(reg_val, fpga_map_addr + REG_SDP3430_FPGA_GPIO_2);
		/* give a few milli sec to settle down
		 * Let the sensor also settle down.. if required..
		 */
		if (enable)
			mdelay(10);
	}

	if (enable) {
		reg_val |= FPGA_SPR_GPIO1_3v3 | FPGA_GPIO6_DIR_CTRL;
		writew(reg_val, fpga_map_addr + REG_SDP3430_FPGA_GPIO_2);
	}
	iounmap(fpga_map_addr);
	/* Vrise time for the voltage - should be less than 1 ms */
	mdelay(1);
}
#endif

#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
#ifdef CONFIG_VIDEO_DW9710
static int dw9710_lens_power_set(enum v4l2_power power)
{
	if (!cam_inited) {
		printk(KERN_ERR "DW9710: Unable to control board GPIOs!\n");
		return -EFAULT;
	}

	/* The power change depends on MT9P012 powerup GPIO, so if we request a
	 * power state different from sensor, we should return error
	 */
	if ((mt9p012_previous_power != V4L2_POWER_OFF) &&
					(power != mt9p012_previous_power))
		return -EIO;
	/*
	 * Plug regulator consumer to respective VAUX supply
	 * if not done before.
	 */
	if (!sdp3430_dw9710_reg) {
		sdp3430_dw9710_reg = regulator_get(camkit_dev, "vaux2_2");
		if (IS_ERR(sdp3430_dw9710_reg)) {
			dev_err(camkit_dev, "vaux2_2 regulator missing\n");
			return PTR_ERR(sdp3430_dw9710_reg);
		}
	}

	if (!sdp3430_dw9710_reg) {
		sdp3430_dw9710_reg = regulator_get(camkit_dev, "vaux2_2");
		if (IS_ERR(sdp3430_dw9710_reg)) {
			dev_err(camkit_dev, "vaux2_2 regulator missing\n");
			return PTR_ERR(sdp3430_dw9710_reg);
		}
	}

	switch (power) {
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		if (regulator_is_enabled(sdp3430_dw9710_reg))
			regulator_disable(sdp3430_dw9710_reg);
		enable_fpga_vio_1v8(0);
		break;
	case V4L2_POWER_ON:
		/* STANDBY_GPIO is active HIGH for set LOW to release */
		gpio_set_value(MT9P012_STANDBY_GPIO, 1);

		/* nRESET is active LOW. set HIGH to release reset */
		gpio_set_value(CAMKITV3_RESET_GPIO, 1);

		/* turn on digital power */
		enable_fpga_vio_1v8(1);

		/* turn on analog power */
		regulator_enable(sdp3430_dw9710_reg);

		/* out of standby */
		gpio_set_value(MT9P012_STANDBY_GPIO, 0);
		udelay(1000);

		/* have to put sensor to reset to guarantee detection */
		gpio_set_value(CAMKITV3_RESET_GPIO, 0);

		udelay(1500);

		/* nRESET is active LOW. set HIGH to release reset */
		gpio_set_value(CAMKITV3_RESET_GPIO, 1);
		/* give sensor sometime to get out of the reset.
		 * Datasheet says 2400 xclks. At 6 MHz, 400 usec is
		 * enough
		 */
		udelay(300);
		break;
	case V4L2_POWER_STANDBY:
		break;
	}
	return 0;
}

static int dw9710_lens_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_LENS;

	return 0;
}

struct dw9710_platform_data sdp3430_dw9710_platform_data = {
	.power_set      = dw9710_lens_power_set,
	.priv_data_set  = dw9710_lens_set_prv_data,
};
#endif

#if defined(CONFIG_VIDEO_TPS61059) || defined(CONFIG_VIDEO_TPS61059_MODULE)
static void tps61059_flash_on(void)
{
	gpio_set_value(TPS61059_TORCH_EN_GPIO, 1);
	gpio_set_value(TPS61059_FLASH_STROBE_GPIO, 1);
}

static void tps61059_flash_off(void)
{
	gpio_set_value(TPS61059_FLASH_STROBE_GPIO, 0);
	gpio_set_value(TPS61059_TORCH_EN_GPIO, 0);
}

static void tps61059_s_torch_intensity(u32 value)
{
	if (value > 0) {
		/* Torch mode, light immediately on, duration indefinite */
		gpio_set_value(TPS61059_TORCH_EN_GPIO, 1);
	} else {
		/* Torch mode off */
		gpio_set_value(TPS61059_TORCH_EN_GPIO, 0);
	}
}

static int tps61059_set_prv_data(void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_FLASH;

	return 0;
}

struct tps61059_platform_data sdp3430_tps61059_data = {
	.flash_on		= tps61059_flash_on,
	.flash_off		= tps61059_flash_off,
	.s_torch_intensity	= tps61059_s_torch_intensity,
	.priv_data_set		= tps61059_set_prv_data,
};

#endif

static struct omap34xxcam_sensor_config cam_hwc = {
	.sensor_isp = 0,
	.capture_mem = MT9P012_BIGGEST_FRAME_BYTE_SIZE * 4,
	.ival_default	= { 1, 10 },
};

static int mt9p012_sensor_set_prv_data(struct v4l2_int_device *s, void *priv)
{
	struct omap34xxcam_hw_config *hwc = priv;

	hwc->u.sensor.sensor_isp = cam_hwc.sensor_isp;
	hwc->u.sensor.capture_mem = cam_hwc.capture_mem;
	hwc->dev_index = 0;
	hwc->dev_minor = 0;
	hwc->dev_type = OMAP34XXCAM_SLAVE_SENSOR;
	return 0;
}

static struct isp_interface_config mt9p012_if_config = {
	.ccdc_par_ser = ISP_PARLL,
	.dataline_shift = 0x1,
	.hsvs_syncdetect = ISPCTRL_SYNC_DETECT_VSRISE,
	.strobe = 0x0,
	.prestrobe = 0x0,
	.shutter = 0x0,
	.prev_sph = 2,
	.prev_slv = 0,
	.wenlog = ISPCCDC_CFG_WENLOG_AND,
	.wait_hs_vs = 2,
	.u.par.par_bridge = 0x0,
	.u.par.par_clk_pol = 0x0,
};

static int mt9p012_sensor_power_set(struct v4l2_int_device *s,
				    enum v4l2_power power)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;

	if (!cam_inited) {
		printk(KERN_ERR "MT9P012: Unable to control board GPIOs!\n");
		return -EFAULT;
	}

	/*
	 * Plug regulator consumer to respective VAUX supply
	 * if not done before.
	 */
	if (!sdp3430_mt9p012_reg) {
		sdp3430_mt9p012_reg = regulator_get(camkit_dev, "vaux2_1");
		if (IS_ERR(sdp3430_mt9p012_reg)) {
			dev_err(camkit_dev, "vaux2_1 regulator missing\n");
			return PTR_ERR(sdp3430_mt9p012_reg);
		}
	}

	switch (power) {
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		if (regulator_is_enabled(sdp3430_mt9p012_reg))
			regulator_disable(sdp3430_mt9p012_reg);
		enable_fpga_vio_1v8(0);

#ifdef CONFIG_OMAP_PM_SRF
		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 0);
#endif
		break;
	case V4L2_POWER_ON:

#ifdef CONFIG_OMAP_PM_SRF
		/* Through-put requirement:
		 * 2592 x 1944 x 2Bpp x 11fps x 3 memory ops = 324770 KByte/s
		 */
		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 324770);
#endif

		if (mt9p012_previous_power == V4L2_POWER_OFF) {
			/* Power Up Sequence */
			isp_configure_interface(vdev->cam->isp,
						&mt9p012_if_config);

			/* set to output mode */
			gpio_direction_output(MT9P012_STANDBY_GPIO, true);
			/* set to output mode */
			gpio_direction_output(CAMKITV3_RESET_GPIO, true);

			/* STANDBY_GPIO is active HIGH for set LOW to release */
			gpio_set_value(MT9P012_STANDBY_GPIO, 1);

			/* nRESET is active LOW. set HIGH to release reset */
			gpio_set_value(CAMKITV3_RESET_GPIO, 1);

			/* turn on digital power */
			enable_fpga_vio_1v8(1);

			/* turn on analog power */
			regulator_enable(sdp3430_mt9p012_reg);
		}

		/* out of standby */
		gpio_set_value(MT9P012_STANDBY_GPIO, 0);
		udelay(1000);

		if (mt9p012_previous_power == V4L2_POWER_OFF) {
			/* have to put sensor to reset to guarantee detection */
			gpio_set_value(CAMKITV3_RESET_GPIO, 0);

			udelay(1500);

			/* nRESET is active LOW. set HIGH to release reset */
			gpio_set_value(CAMKITV3_RESET_GPIO, 1);
			/* give sensor sometime to get out of the reset.
			 * Datasheet says 2400 xclks. At 6 MHz, 400 usec is
			 * enough
			 */
			udelay(300);
		}
		break;
	case V4L2_POWER_STANDBY:
		/* stand by */
		gpio_set_value(MT9P012_STANDBY_GPIO, 1);
#ifdef CONFIG_OMAP_PM_SRF
		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 0);
#endif
		break;
	}
	/* Save powerstate to know what was before calling POWER_ON. */
	mt9p012_previous_power = power;
	return 0;
}

static u32 mt9p012_sensor_set_xclk(struct v4l2_int_device *s, u32 xclkfreq)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;

	return isp_set_xclk(vdev->cam->isp, xclkfreq, CAMKITV3_USE_XCLKA);
}

struct mt9p012_platform_data sdp3430_mt9p012_platform_data = {
	.power_set      = mt9p012_sensor_power_set,
	.priv_data_set  = mt9p012_sensor_set_prv_data,
	.set_xclk       = mt9p012_sensor_set_xclk,
};

#endif

#if defined(CONFIG_VIDEO_OV3640) || defined(CONFIG_VIDEO_OV3640_MODULE)

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
	.prev_slv = 0,
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
	hwc->u.sensor.capture_mem = ov3640_hwc.capture_mem;
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

	/*
	 * Plug regulator consumer to respective VAUX supply
	 * if not done before.
	 */
	if (!sdp3430_ov3640_reg) {
#if defined(CONFIG_VIDEO_OV3640_CSI2)
		sdp3430_ov3640_reg = regulator_get(camkit_dev, "vaux4_1");
		if (IS_ERR(sdp3430_ov3640_reg)) {
			dev_err(camkit_dev, "vaux4_1 regulator missing\n");
			return PTR_ERR(sdp3430_ov3640_reg);
		}
#else
		sdp3430_ov3640_reg = regulator_get(camkit_dev, "vaux2_3");
		if (IS_ERR(sdp3430_ov3640_reg)) {
			dev_err(camkit_dev, "vaux2_3 regulator missing\n");
			return PTR_ERR(sdp3430_ov3640_reg);
		}
#endif
	}

	switch (power) {
	case V4L2_POWER_ON:
#ifdef CONFIG_OMAP_PM_SRF
		/* Through-put requirement:
		 * 2048 x 1536 x 2Bpp x 7.5fps x 3 memory ops = 138240 KByte/s
		 */
		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 138240);
#endif
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
			regulator_enable(sdp3430_ov3640_reg);
			udelay(100);

			/* Turn ON Omnivision sensor */
			gpio_set_value(CAMKITV3_RESET_GPIO, 1);
			gpio_set_value(OV3640_STANDBY_GPIO, 0);
			udelay(100);

			/* RESET Omnivision sensor */
			gpio_set_value(CAMKITV3_RESET_GPIO, 0);
			udelay(100);
			gpio_set_value(CAMKITV3_RESET_GPIO, 1);

			/* Wait 10 ms */
			mdelay(10);
			enable_fpga_vio_1v8(1);
			udelay(100);
		}
		break;
	case V4L2_POWER_OFF:
		/* Power Down Sequence */
		isp_csi2_complexio_power(ISP_CSI2_POWER_OFF);
		if (regulator_is_enabled(sdp3430_ov3640_reg))
			regulator_disable(sdp3430_ov3640_reg);
		enable_fpga_vio_1v8(0);
#ifdef CONFIG_OMAP_PM_SRF
		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 0);
#endif
		break;
	case V4L2_POWER_STANDBY:
#ifdef CONFIG_OMAP_PM_SRF
		omap_pm_set_min_bus_tput(vdev->cam->isp, OCP_INITIATOR_AGENT, 0);
#endif
		break;
	}
	previous_power = power;
	return 0;
}

static u32 ov3640_sensor_set_xclk(struct v4l2_int_device *s, u32 xclkfreq)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;

	return isp_set_xclk(vdev->cam->isp, xclkfreq, CAMKITV3_USE_XCLKB);
}

struct ov3640_platform_data sdp3430_ov3640_platform_data = {
	.power_set	 = ov3640_sensor_power_set,
	.priv_data_set	 = ov3640_sensor_set_prv_data,
	.set_xclk	 = ov3640_sensor_set_xclk,
};

#endif

static int sdp3430_camkit_probe(struct platform_device *pdev)
{
	int ret = 0;

	if (!is_mt9p012_enabled() && !is_ov3640_enabled())
		return -ENODEV;

	/* Request and configure shared gpio pins for both cameras */
	if (is_mt9p012_enabled() || is_ov3640_enabled()) {
		if (gpio_request(CAMKITV3_RESET_GPIO,
				 "camkitv3_reset_gpio") != 0) {
			dev_err(&pdev->dev, "Could not request GPIO %d",
				CAMKITV3_RESET_GPIO);
			ret = -ENODEV;
			goto err;
		}
		gpio_direction_output(CAMKITV3_RESET_GPIO, false);
	}

	/* Request and configure shared gpio pins for primary camera */
	if (is_mt9p012_enabled()) {
		if (gpio_request(MT9P012_STANDBY_GPIO,
				 "mt9p012_standby_gpio")) {
			dev_err(&pdev->dev,
				"Could not request GPIO %d for MT9P012\n",
				MT9P012_STANDBY_GPIO);
			ret = -ENODEV;
			goto err_freegpio1;
		}

		gpio_direction_output(MT9P012_STANDBY_GPIO, false);

		if (is_tps61059_enabled()) {
			/* Configure pin MUX for GPIO 126 for TPS61059 flash */
			omap_cfg_reg(D25_34XX_GPIO126_OUT);

			if (gpio_request(TPS61059_TORCH_EN_GPIO,
					 "tps61059_torch_en_gpio")) {
				dev_err(&pdev->dev,
					"Could not request GPIO %d for"
					" TPS61059\n",
					TPS61059_TORCH_EN_GPIO);
				ret = -ENODEV;
				goto err_freegpio2;
			}

			if (gpio_request(TPS61059_FLASH_STROBE_GPIO,
					 "tps61059_flash_strobe_gpio")) {
				dev_err(&pdev->dev,
					"Could not request GPIO %d for"
					" TPS61059\n",
					TPS61059_FLASH_STROBE_GPIO);
				ret = -ENODEV;
				goto err_freegpio3;
			}
			gpio_direction_output(TPS61059_TORCH_EN_GPIO, false);
			gpio_direction_output(TPS61059_FLASH_STROBE_GPIO,
					      false);
		}
	}

	/* Request and configure shared gpio pins for secondary camera */
	if (is_ov3640_enabled()) {
		if (gpio_request(OV3640_STANDBY_GPIO,
				 "ov3640_standby_gpio") != 0) {
			dev_err(&pdev->dev, "Could not request GPIO %d",
				OV3640_STANDBY_GPIO);
			ret = -ENODEV;
			goto err_freegpio4;
		}
		gpio_direction_output(OV3640_STANDBY_GPIO, false);
	}

	cam_inited = 1;
	camkit_dev = &pdev->dev;
	return 0;

err_freegpio4:
	if (is_mt9p012_enabled() && is_tps61059_enabled())
		gpio_free(TPS61059_FLASH_STROBE_GPIO);
err_freegpio3:
	if (is_mt9p012_enabled() && is_tps61059_enabled())
		gpio_free(TPS61059_TORCH_EN_GPIO);
err_freegpio2:
	if (is_mt9p012_enabled())
		gpio_free(MT9P012_STANDBY_GPIO);
err_freegpio1:
	if (is_mt9p012_enabled() || is_ov3640_enabled())
		gpio_free(CAMKITV3_RESET_GPIO);
err:
	cam_inited = 0;
	return ret;
}

static int sdp3430_camkit_remove(struct platform_device *pdev)
{
	/* Primary camera resources */
	if (is_mt9p012_enabled()) {
		/* Free Regulators */
		if (is_dw9710_enabled()) {
			if (regulator_is_enabled(sdp3430_dw9710_reg))
				regulator_disable(sdp3430_dw9710_reg);
			regulator_put(sdp3430_dw9710_reg);
		}

		if (regulator_is_enabled(sdp3430_mt9p012_reg))
			regulator_disable(sdp3430_mt9p012_reg);
		regulator_put(sdp3430_mt9p012_reg);

		/* Free GPIOs */
		if (is_tps61059_enabled()) {
			gpio_free(TPS61059_FLASH_STROBE_GPIO);
			gpio_free(TPS61059_TORCH_EN_GPIO);
		}
		gpio_free(MT9P012_STANDBY_GPIO);
	}

	/* Secondary camera resources */
	if (is_ov3640_enabled()) {
		/* Free Regulators */
		if (regulator_is_enabled(sdp3430_ov3640_reg))
			regulator_disable(sdp3430_ov3640_reg);
		regulator_put(sdp3430_ov3640_reg);

		/* Free GPIOs */
		gpio_free(OV3640_STANDBY_GPIO);
	}

	/* Shared resources to free */
	if (is_mt9p012_enabled() || is_ov3640_enabled())
		gpio_free(CAMKITV3_RESET_GPIO);
	return 0;
}

static int sdp3430_camkit_suspend(struct device *dev)
{
	return 0;
}

static int sdp3430_camkit_resume(struct device *dev)
{
	return 0;
}

static struct dev_pm_ops sdp3430_camkit_pm_ops = {
	.suspend = sdp3430_camkit_suspend,
	.resume  = sdp3430_camkit_resume,
};

static struct platform_driver sdp3430_camkit_driver = {
	.probe		= sdp3430_camkit_probe,
	.remove		= sdp3430_camkit_remove,
	.driver		= {
		.name	= "sdp3430_camkit",
		.pm	= &sdp3430_camkit_pm_ops,
	},
};

void __init sdp3430_cam_init(void)
{
	cam_inited = 0;
	platform_driver_register(&sdp3430_camkit_driver);
}

