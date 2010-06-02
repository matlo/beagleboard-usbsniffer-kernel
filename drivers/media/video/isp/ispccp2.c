/*
 * ispccp2.c
 *
 * Driver Library for CCP2 module in TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2010 Nokia Corporation.
 * Copyright (C) 2010 Texas Instruments, Inc.
 *
 * Contributors:
 *     RaniSuneela <r-m@ti.com>
 *
 * Based on code by:
 *   Contributors of isp driver:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Mohit Jalori <mjalori@ti.com>
 * 	Sergio Aguirre <saaguirre@ti.com>
 * 	Sakari Ailus <sakari.ailus@nokia.com>
 * 	Tuukka Toivonen <tuukka.o.toivonen@nokia.com>
 *	Toni Leinonen <toni.leinonen@nokia.com>
 *	David Cohen <david.cohen@nokia.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/mm.h>

#include "isp.h"
#include "ispreg.h"
#include "ispccp2.h"

/* Number of LCX channels */
#define CCP2_LCx_CHANS_NUM			3
#define ISP_BITS_PER_PIXEL			16
/* Max/Min size for CCP2 video port */
#define ISPCCP2_DAT_START_MIN			0
#define ISPCCP2_DAT_START_MAX			4095
#define ISPCCP2_DAT_SIZE_MIN			0
#define ISPCCP2_DAT_SIZE_MAX			4095
#define ISPCCP2_VPCLK_FRACDIV			65536
#define ISPCCP2_LCx_CTRL_FORMAT_RAW8_DPCM10_VP	0x12
#define ISPCCP2_LCx_CTRL_FORMAT_RAW10_VP	0x16
/* Max/Min size for CCP2 memory channel */
#define ISPCCP2_LCM_HSIZE_COUNT_MIN		16
#define ISPCCP2_LCM_HSIZE_COUNT_MAX		8191
#define ISPCCP2_LCM_HSIZE_SKIP_MIN		0
#define ISPCCP2_LCM_HSIZE_SKIP_MAX		8191
#define ISPCCP2_LCM_VSIZE_MIN			1
#define ISPCCP2_LCM_VSIZE_MAX			8191
#define ISPCCP2_LCM_HWORDS_MIN			1
#define ISPCCP2_LCM_HWORDS_MAX			4095
#define ISPCCP2_LCM_CTRL_BURST_SIZE_32X		5
#define ISPCCP2_LCM_CTRL_READ_THROTTLE_FULL	0
#define ISPCCP2_LCM_CTRL_SRC_FORMAT_RAW10	3
#define ISPCCP2_LCM_CTRL_DST_FORMAT_RAW10	3
#define ISPCCP2_LCM_CTRL_DST_PORT_VP		0
#define ISPCCP2_LCM_CTRL_DST_PORT_MEM		1

/* Set only the required bits */
#define BIT_SET(var, shift, mask, val)		\
	do {					\
		var = (var & ~(mask << shift))	\
			| (val << shift);	\
	} while (0)

/* Structure for saving/restoring ccp2 module registers */
/* Saving/Restoring only registers modified here */
static struct isp_reg ispccp2_reg_list[] = {
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_SYSCONFIG, 0x0000},
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LC01_IRQENABLE, 0x0000},
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_CTRL, 0x0000},
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCx_CTRL(0), 0x0000},
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCx_DAT_START(0), 0x0000},
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCx_DAT_SIZE(0), 0x0000},
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCx_DAT_OFST(0), 0x0000},
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_CTRL, 0x0000},
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_VSIZE, 0x0000},
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_HSIZE, 0x0000},
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_PREFETCH, 0x0000},
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_SRC_ADDR, 0x0000},
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_SRC_OFST, 0x0000},
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_DST_ADDR, 0x0000},
	{OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_DST_OFST, 0x0000},
	{0, ISP_TOK_TERM, 0x0000}
};

/*
 * ispccp2_save_context - Saves values of ccp2 registers.
 */
void ispccp2_save_context(struct isp_device *isp)
{
	dev_dbg(isp->dev, "Saving ccp2 context\n");
	isp_save_context(isp, ispccp2_reg_list);
}

/*
 * ispccp2_restore_context - Restores ccp2 register values.
 */
void ispccp2_restore_context(struct isp_device *isp)
{
	dev_dbg(isp->dev, "Restoring ccp2 context\n");
	isp_restore_context(isp, ispccp2_reg_list);
}

/*
 * ispccp2_reset - Reset the CCP2
 * @isp: pointer to isp device
 */
static void ispccp2_reset(struct isp_device *isp)
{
	int i = 0;

	/* Reset the CSI1/CCP2B and wait for reset to complete */
	isp_reg_or(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_SYSCONFIG,
		   ISPCCP2_SYSCONFIG_SOFT_RESET);
	while (!(isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_SYSSTATUS) &
		 ISPCCP2_SYSSTATUS_RESET_DONE)) {
		udelay(10);
		if (i++ > 10) {  /* try read 10 times */
			dev_warn(isp->dev,
				"omap3_isp: timeout waiting for ccp2 reset\n");
			break;
		}
	}
}

/*
 * ispccp2_if_enable - Enable CCP2 interface.
 * @isp: pointer to isp device
 * @enable: enable/disable flag
 */
static void ispccp2_if_enable(struct isp_device *isp, u8 enable)
{
	int i;

	/* Enable/Disable all the LCx channels */
	for (i = 0; i < CCP2_LCx_CHANS_NUM; i++)
		isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCx_CTRL(i),
			       ~(ISPCCP2_LCx_CTRL_CHAN_EN),
			       enable ? ISPCCP2_LCx_CTRL_CHAN_EN : 0);

	/* Enable/Disable ccp2 interface in ccp2 mode */
	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_CTRL,
		       ~(ISPCCP2_CTRL_MODE | ISPCCP2_CTRL_IF_EN),
		       enable ? (ISPCCP2_CTRL_MODE | ISPCCP2_CTRL_IF_EN) : 0);
}

/*
 * ispccp2_mem_enable - Enable CCP2 memory interface.
 * @isp: pointer to isp device
 * @enable: enable/disable flag
 */
static void ispccp2_mem_enable(struct isp_ccp2_device *ccp2, u8 enable)
{
	struct isp_device *isp = to_isp_device(ccp2);

	/* Enable/Disable ccp2 interface in ccp2 mode */
	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_CTRL,
		       ~ISPCCP2_CTRL_MODE, enable ? ISPCCP2_CTRL_MODE : 0);

	if (enable)
		ispccp2_if_enable(isp, 0);

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_CTRL,
		       ~ISPCCP2_LCM_CTRL_CHAN_EN,
		       enable ? ISPCCP2_LCM_CTRL_CHAN_EN : 0);
}

/*
 * ispccp2_phyif_config - Initialize CCP2 phy interface config
 * @isp: Pointer to ISP device structure.
 * @config: CCP2 platform data
 *
 * Configure the CCP2 physical interface module from platform data.
 *
 * Returns -EIO if strobe is chosen in CSI1 mode, or 0 on success.
 */
static int ispccp2_phyif_config(struct isp_device *isp,
			struct isp_ccp2_platform_data *pdata)
{
	u32 val;

	/* CCP2B mode */
	val = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_CTRL) |
			    ISPCCP2_CTRL_IO_OUT_SEL | ISPCCP2_CTRL_MODE;
	/* Data/strobe physical layer */
	BIT_SET(val, ISPCCP2_CTRL_PHY_SEL_SHIFT, ISPCCP2_CTRL_PHY_SEL_MASK,
		pdata->phy_layer);
	BIT_SET(val, ISPCCP2_CTRL_INV_SHIFT, ISPCCP2_CTRL_INV_MASK,
		pdata->strobe_clk_pol);
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_CTRL);

	val = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_CTRL);
	if (!(val & ISPCCP2_CTRL_MODE)) {
		if (pdata->ccp2_mode)
			dev_warn(isp->dev, "OMAP3 CCP2 bus not available\n");
		if (pdata->phy_layer == ISPCCP2_CTRL_PHY_SEL_STROBE)
			/* Strobe mode requires CCP2 */
			return -EIO;
	}

	return 0;
}

/*
 * ispccp2_vp_config - Initialize CCP2 video port interface.
 * @isp: Pointer to ISP device structure.
 * @config: Pointer to ISP vp interface config structure.
 *
 * This will analyze the parameters passed by the interface config
 * and configure CCP2 video port.
 */
static void ispccp2_vp_config(struct isp_device *isp,
			struct isp_ccp2_platform_data *pdata)
{
	u32 val;

	/* ISPCCP2_CTRL Video port */
	val = (isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_CTRL)
			| (ISPCCP2_CTRL_VP_CLK_POL | ISPCCP2_CTRL_MODE));
	val &= ~ISPCCP2_CTRL_VP_ONLY_EN;	/* Enable VP only off */

	if (isp->revision == ISP_REVISION_15_0)
		BIT_SET(val, ISPCCP2_CTRL_VPCLK_DIV_SHIFT,
			ISPCCP2_CTRL_VPCLK_DIV_MASK,
			ISPCCP2_VPCLK_FRACDIV / (pdata->vpclk_div + 1));
	else
		BIT_SET(val, ISPCCP2_CTRL_VP_OUT_CTRL_SHIFT,
			ISPCCP2_CTRL_VP_OUT_CTRL_MASK,
			pdata->vpclk_div);

	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_CTRL);
}

/*
 * ispccp2_lcx_config - Initialize CCP2 logical channel interface.
 * @isp: Pointer to ISP device structure.
 * @config: Pointer to ISP LCx config structure.
 *
 * This will analyze the parameters passed by the interface config
 * and configure CSI1/CCP2 logical channel
 *
 */
static void ispccp2_lcx_config(struct isp_device *isp,
			       struct isp_interface_lcx_config *config)
{
	u32 val, format;

	switch (config->format) {
	case V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8:
		format = ISPCCP2_LCx_CTRL_FORMAT_RAW8_DPCM10_VP;
		break;
	case V4L2_MBUS_FMT_SGRBG10_1X10:
	default:
		format = ISPCCP2_LCx_CTRL_FORMAT_RAW10_VP;	/* RAW10+VP */
		break;
	}
	/* ISPCCP2_LCx_CTRL logical channel #0 */
	val = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCx_CTRL(0))
			    | (ISPCCP2_LCx_CTRL_REGION_EN); /* Region */

	if (isp->revision == ISP_REVISION_15_0) {
		/* CRC */
		BIT_SET(val, ISPCCP2_LCx_CTRL_CRC_SHIFT_15_0,
			ISPCCP2_LCx_CTRL_CRC_MASK,
			config->crc);
		/* Format = RAW10+VP or RAW8+DPCM10+VP*/
		BIT_SET(val, ISPCCP2_LCx_CTRL_FORMAT_SHIFT_15_0,
			ISPCCP2_LCx_CTRL_FORMAT_MASK_15_0, format);
	} else {
		BIT_SET(val, ISPCCP2_LCx_CTRL_CRC_SHIFT,
			ISPCCP2_LCx_CTRL_CRC_MASK,
			config->crc);

		BIT_SET(val, ISPCCP2_LCx_CTRL_FORMAT_SHIFT,
			ISPCCP2_LCx_CTRL_FORMAT_MASK, format);
	}
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCx_CTRL(0));

	/* ISPCCP2_DAT_START for logical channel #0 */
	isp_reg_writel(isp, config->data_start << ISPCCP2_LCx_DAT_SHIFT,
		       OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCx_DAT_START(0));

	/* ISPCCP2_DAT_SIZE for logical channel #0 */
	isp_reg_writel(isp, config->data_size << ISPCCP2_LCx_DAT_SHIFT,
		       OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCx_DAT_SIZE(0));

	/* Clear status bits for logical channel #0 */
	val = ISPCCP2_LC01_IRQSTATUS_LC0_FIFO_OVF_IRQ |
	      ISPCCP2_LC01_IRQSTATUS_LC0_CRC_IRQ |
	      ISPCCP2_LC01_IRQSTATUS_LC0_FSP_IRQ |
	      ISPCCP2_LC01_IRQSTATUS_LC0_FW_IRQ |
	      ISPCCP2_LC01_IRQSTATUS_LC0_FSC_IRQ |
	      ISPCCP2_LC01_IRQSTATUS_LC0_SSC_IRQ;

	/* Clear IRQ status bits for logical channel #0 */
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_CCP2,
		       ISPCCP2_LC01_IRQSTATUS);
}

/*
 * ispccp2_mem_in_config - Initialize CCP2 memory input interface
 * @isp: Pointer to ISP device structure
 * @config: Pointer to ISP mem interface config structure
 *
 * This will analyze the parameters passed by the interface config
 * structure, and configure the respective registers for proper
 * CSI1/CCP2 memory input.
 */
static void ispccp2_mem_in_config(struct isp_device *isp,
			struct isp_interface_mem_config *config)
{
	u32 val, hwords;

	isp_reg_writel(isp, (ISPCSI1_MIDLEMODE_SMARTSTANDBY <<
		       ISPCSI1_MIDLEMODE_SHIFT),
		       OMAP3_ISP_IOMEM_CCP2, ISP_CSIB_SYSCONFIG);
	isp_reg_or(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_CTRL,
		   ISPCCP2_CTRL_IO_OUT_SEL | ISPCCP2_CTRL_MODE);
	/* Burst size to 32x64 */
	val = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_CTRL);
	BIT_SET(val, ISPCCP2_LCM_CTRL_BURST_SIZE_SHIFT,
		ISPCCP2_LCM_CTRL_BURST_SIZE_MASK,
		ISPCCP2_LCM_CTRL_BURST_SIZE_32X);
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_CTRL);
	/* Hsize, Skip */
	isp_reg_writel(isp, ISPCCP2_LCM_HSIZE_SKIP_MIN |
		       (config->hsize_count << ISPCCP2_LCM_HSIZE_SHIFT),
		       OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_HSIZE);

	isp_reg_writel(isp, config->src_ofst, OMAP3_ISP_IOMEM_CCP2,
		       ISPCCP2_LCM_SRC_OFST);
	/* Src format */
	val = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_CTRL);
	BIT_SET(val, ISPCCP2_LCM_CTRL_SRC_FORMAT_SHIFT,
		ISPCCP2_LCM_CTRL_SRC_FORMAT_MASK,
		ISPCCP2_LCM_CTRL_SRC_FORMAT_RAW10);
	/* Destination format */
	BIT_SET(val, ISPCCP2_LCM_CTRL_DST_FORMAT_SHIFT,
		ISPCCP2_LCM_CTRL_DST_FORMAT_MASK,
		ISPCCP2_LCM_CTRL_DST_FORMAT_RAW10);
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_CTRL);
	/* Vsize, no. of lines */
	isp_reg_writel(isp, config->vsize_count << ISPCCP2_LCM_VSIZE_SHIFT,
		       OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_VSIZE);
	/* Prefetch, to use ceil value */
	hwords = 4 * (((ISPCCP2_LCM_HSIZE_SKIP_MIN + config->hsize_count)
		 * ISP_BITS_PER_PIXEL) / 256);
	isp_reg_writel(isp, hwords << ISPCCP2_LCM_PREFETCH_SHIFT,
		       OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_PREFETCH);

	/* VP CLK */
	val = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_CTRL);
	BIT_SET(val, ISPCCP2_CTRL_VPCLK_DIV_SHIFT, ISPCCP2_CTRL_VPCLK_DIV_MASK,
		ISPCCP2_VPCLK_FRACDIV / 2);
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_CTRL);

	/* Clear LCM interrupts */
	isp_reg_writel(isp, ISPCCP2_LCM_IRQSTATUS_OCPERROR_IRQ |
		       ISPCCP2_LCM_IRQSTATUS_EOF_IRQ,
		       OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_IRQSTATUS);

	/* Enable LCM interupts */
	isp_reg_or(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_IRQENABLE,
		   (ISPCCP2_LCM_IRQSTATUS_EOF_IRQ |
		    ISPCCP2_LCM_IRQSTATUS_OCPERROR_IRQ));
}

/*
 * ispccp2_set_inaddr - Sets memory address of input frame.
 * @addr: 32bit memory address aligned on 32byte boundary.
 *
 * Configures the memory address from which the input frame is to be read.
 */
static void ispccp2_set_inaddr(struct isp_ccp2_device *ccp2, u32 addr)
{
	struct isp_device *isp = to_isp_device(ccp2);

	isp_reg_writel(isp, addr, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LCM_SRC_ADDR);
}

/* -----------------------------------------------------------------------------
 * V4L2 subdev operations
 */

const static unsigned int ccp2_fmts[] = {
	V4L2_MBUS_FMT_SGRBG10_1X10,
	V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8,
};

/*
 * ispccp2_enum_mbus_code - Handle pixel format enumeration
 * @sd     : pointer to v4l2 subdev structure
 * @code   : pointer to v4l2_subdev_pad_mbus_code_enum structure
 * return -EINVAL or zero on success
 */
static int ispccp2_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_mbus_code_enum *code)
{
	if (code->pad >= CCP2_PADS_NUM ||
	    code->index >= ARRAY_SIZE(ccp2_fmts))
		return -EINVAL;

	code->code = ccp2_fmts[code->index];
	return 0;
}

/*
 * __ispccp2_get_format - helper function for getting ccp2 format
 * @ccp2  : pointer to ccp2 structure
 * @pad   : pad number
 * @which : wanted subdev format
 * return format structure or NULL on error
 */
static struct v4l2_mbus_framefmt *
__ispccp2_get_format(struct isp_ccp2_device *ccp2, unsigned int pad,
		enum v4l2_subdev_format which)
{
	if (which != V4L2_SUBDEV_FORMAT_PROBE &&
	    which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return NULL;

	if (pad >= CCP2_PADS_NUM)
		return NULL;

	return &ccp2->formats[pad][which];
}

/*
 * ispccp2_get_format - Handle get format by pads subdev method
 * @sd    : pointer to v4l2 subdev structure
 * @pad   : pad num
 * @fmt   : pointer to v4l2 mbus format structure
 * @which : wanted subdev format
 * return -EINVAL or zero on sucess
 */
static int ispccp2_get_format(struct v4l2_subdev *sd, unsigned int pad,
			struct v4l2_mbus_framefmt *fmt,
			enum v4l2_subdev_format which)
{
	struct isp_ccp2_device *ccp2 = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format =
			__ispccp2_get_format(ccp2, pad, which);

	if (format == NULL)
		return -EINVAL;

	memcpy(fmt, format, sizeof(*fmt));

	return 0;
}

/*
 * ispccp2_try_format - Handle try format by pad subdev method
 * @sd    : pointer to v4l2 subdev structure
 * @pad   : pad num
 * @fmt   : pointer to v4l2 mbus format structure
 * @which : wanted subdev format
 */
static void ispccp2_try_format(struct v4l2_subdev *sd, unsigned int pad,
				struct v4l2_mbus_framefmt *fmt,
				enum v4l2_subdev_format which)
{
	struct isp_ccp2_device *ccp2 = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	switch (pad) {
	case CCP2_PAD_SINK:
		if (fmt->code != V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8)
			fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;

		if (ccp2->input == CCP2_INPUT_SENSOR) {
			fmt->width = clamp_t(u32, fmt->width,
					     ISPCCP2_DAT_START_MIN,
					     ISPCCP2_DAT_START_MAX);
			fmt->height = clamp_t(u32, fmt->height,
					      ISPCCP2_DAT_SIZE_MIN,
					      ISPCCP2_DAT_SIZE_MAX);
		} else if (ccp2->input == CCP2_INPUT_MEMORY) {
			fmt->width = clamp_t(u32, fmt->width,
					     ISPCCP2_LCM_HSIZE_COUNT_MIN,
					     ISPCCP2_LCM_HSIZE_COUNT_MAX);
			fmt->width &= ~15;
			fmt->height = clamp_t(u32, fmt->height,
					      ISPCCP2_LCM_VSIZE_MIN,
					      ISPCCP2_LCM_VSIZE_MAX);
		}
		break;

	case CCP2_PAD_SOURCE:
		/* Source format same as sink's format */
		format = __ispccp2_get_format(ccp2, CCP2_PAD_SINK, which);
		memcpy(fmt, format, sizeof(*fmt));
		break;
	}

	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
}

/*
 * ispccp2_set_format - Handle set format by pads subdev method
 * @sd    : pointer to v4l2 subdev structure
 * @pad   : pad num
 * @fmt   : pointer to v4l2 mbus format structure
 * @which : wanted subdev format
 * returns zero
 */
static int ispccp2_set_format(struct v4l2_subdev *sd, unsigned int pad,
			struct v4l2_mbus_framefmt *fmt,
			enum v4l2_subdev_format which)
{
	struct isp_ccp2_device *ccp2 = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format =
				__ispccp2_get_format(ccp2, pad, which);

	ispccp2_try_format(sd, pad, fmt, which);
	memcpy(format, fmt, sizeof(*format));

	if (which == V4L2_SUBDEV_FORMAT_PROBE)
		return 0;

	switch (pad) {
	case CCP2_PAD_SOURCE:
		ccp2->if_cfg.format = fmt->code;
		ccp2->if_cfg.data_size = fmt->height;
		if (ccp2->input == CCP2_INPUT_MEMORY) {
			ccp2->mem_cfg.hsize_count = fmt->width;
			ccp2->mem_cfg.vsize_count = fmt->height;
			ccp2->mem_cfg.src_ofst = fmt->width
						* ISP_BYTES_PER_PIXEL;
		}
		break;
	case CCP2_PAD_SINK:
	default:
		break;
	}

	return 0;
}


/*
 * isp_ccp2_configure - Configure ccp2 with data from sensor
 * @ccp2: ISP ccp2 V4L2 subdevice
 *
 * Return 0 on success or a negative error code
 */
static int isp_ccp2_configure(struct isp_ccp2_device *ccp2)
{
	struct isp_device *isp = to_isp_device(ccp2);
	struct media_entity_pad *pad;
	struct v4l2_subdev *sensor;
	u32 lines = 0;
	int ret;

	ret = ispccp2_phyif_config(isp, &isp->pdata->ccp2);
	if (ret < 0)
		return ret;

	ispccp2_vp_config(isp, &isp->pdata->ccp2);

	pad = media_entity_remote_pad(&ccp2->pads[CCP2_PAD_SINK]);
	sensor = media_entity_to_v4l2_subdev(pad->entity);
	v4l2_subdev_call(sensor, sensor, g_skip_top_lines, &lines);

	ccp2->if_cfg.data_start = lines;
	ccp2->if_cfg.crc = isp->pdata->ccp2.crc;
	ispccp2_lcx_config(isp, &ccp2->if_cfg);

	return 0;
}

/*
 * ccp2_set_power - Power on/off the CCP2 module
 * @sd: ISP CCP2 V4L2 subdevice
 * @on: power on/off
 *
 * Return 0 on success or a negative error code otherwise.
 */
static int ccp2_set_power(struct v4l2_subdev *sd, int on)
{
	struct isp_ccp2_device *ccp2 = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(ccp2);

	if (on) {
		if (isp_get(isp) == NULL)
			return -EBUSY;
	} else {
		isp_put(isp);
	}

	return 0;
}

/*
 * ispccp2_s_stream - Enable/Disable streaming on ccp2 subdev
 * @sd    : pointer to v4l2 subdev structure
 * @enable: 1 == Enable, 0 == Disable
 * return zero
 */
static int ispccp2_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct isp_ccp2_device *ccp2 = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(ccp2);
	int ret;

	switch (enable) {
	case ISP_PIPELINE_STREAM_CONTINUOUS:
		if (ccp2->phy) {
			ret = isp_csiphy_acquire(ccp2->phy);
			if (ret < 0)
				return ret;
		}

		isp_ccp2_configure(ccp2);

		/* Enable LCx interrupts */
		isp_reg_writel(isp, IRQ0ENABLE_CCP2_LC0_IRQ,
			       OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LC01_IRQSTATUS);
		isp_reg_or(isp, OMAP3_ISP_IOMEM_CCP2, ISPCCP2_LC01_IRQENABLE,
			   IRQ0ENABLE_CCP2_LC0_IRQ);

		/* ISPCCP2_DAT_START for logical channel #0 */
		isp_reg_writel(isp, ccp2->if_cfg.data_start <<
			       ISPCCP2_LCx_DAT_SHIFT,
			       OMAP3_ISP_IOMEM_CCP2,
			       ISPCCP2_LCx_DAT_START(0));

		/* Enable CSI1/CCP2 interface */
		ispccp2_if_enable(isp, 1);
		break;

	case ISP_PIPELINE_STREAM_SINGLESHOT:
		if (ccp2->state != ISP_PIPELINE_STREAM_SINGLESHOT) {
			isp_sbl_enable(isp, OMAP3_ISP_SBL_CSI1_READ);
			ispccp2_mem_in_config(isp, &ccp2->mem_cfg);
		}
		ispccp2_mem_enable(ccp2, 1);
		break;

	case ISP_PIPELINE_STREAM_STOPPED:
		if (ccp2->input == CCP2_INPUT_MEMORY) {
			ispccp2_mem_enable(ccp2, 0);
			isp_sbl_disable(isp, OMAP3_ISP_SBL_CSI1_READ);
		} else if (ccp2->input == CCP2_INPUT_SENSOR) {
			isp_reg_and(isp, OMAP3_ISP_IOMEM_CCP2,
				    ISPCCP2_LC01_IRQENABLE,
				    ~IRQ0ENABLE_CCP2_LC0_IRQ);

			/* Disable CSI1/CCP2 interface */
			ispccp2_if_enable(isp, 0);
			if (ccp2->phy)
				isp_csiphy_release(ccp2->phy);
		}
		break;
	}

	ccp2->state = enable;
	return 0;
}


/* subdev core operations */
static const struct v4l2_subdev_core_ops ispccp2_sd_core_ops = {
	.s_power = ccp2_set_power,
};

/* subdev video operations */
static const struct v4l2_subdev_video_ops ispccp2_sd_video_ops = {
	.s_stream = ispccp2_s_stream,
};

/* subdev pad operations */
static const struct v4l2_subdev_pad_ops ispccp2_sd_pad_ops = {
	.enum_mbus_code = ispccp2_enum_mbus_code,
	.get_fmt = ispccp2_get_format,
	.set_fmt = ispccp2_set_format,
};

/* subdev operations */
static const struct v4l2_subdev_ops ispccp2_sd_ops = {
	.core = &ispccp2_sd_core_ops,
	.video = &ispccp2_sd_video_ops,
	.pad = &ispccp2_sd_pad_ops,
};


static void ispccp2_isr_buffer(struct isp_ccp2_device *ccp2)
{
	struct isp_buffer *buffer;

	buffer = isp_video_buffer_next(&ccp2->video_in, ccp2->error);
	if (buffer != NULL)
		ispccp2_set_inaddr(ccp2, buffer->isp_addr);

	ccp2->error = 0;
}

/*
 * ispccp2_isr - Handle ISP CCP2 interrupts
 * @isp: Device pointer specific to the OMAP3 ISP.
 *
 * This will handle the CCP2 interrupts
 *
 * Returns -EIO in case of error, or 0 on success.
 */
int ispccp2_isr(struct isp_device *isp)
{
	struct isp_ccp2_device *ccp2 = &isp->isp_ccp2;
	int ret = 0;
	static const u32 ISPCCP2_LC01_ERROR =
		ISPCCP2_LC01_IRQSTATUS_LC0_FIFO_OVF_IRQ |
		ISPCCP2_LC01_IRQSTATUS_LC0_CRC_IRQ |
		ISPCCP2_LC01_IRQSTATUS_LC0_FSP_IRQ |
		ISPCCP2_LC01_IRQSTATUS_LC0_FW_IRQ |
		ISPCCP2_LC01_IRQSTATUS_LC0_FSC_IRQ |
		ISPCCP2_LC01_IRQSTATUS_LC0_SSC_IRQ;
	u32 lcx_irqstatus, lcm_irqstatus;

	/* First clear the interrupts */
	lcx_irqstatus = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCP2,
				      ISPCCP2_LC01_IRQSTATUS);
	isp_reg_writel(isp, lcx_irqstatus, OMAP3_ISP_IOMEM_CCP2,
		       ISPCCP2_LC01_IRQSTATUS);

	lcm_irqstatus = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCP2,
				      ISPCCP2_LCM_IRQSTATUS);
	isp_reg_writel(isp, lcm_irqstatus, OMAP3_ISP_IOMEM_CCP2,
		       ISPCCP2_LCM_IRQSTATUS);
	/* Errors */
	if (lcx_irqstatus & ISPCCP2_LC01_ERROR) {
		ccp2->error = 1;
		dev_dbg(isp->dev, "CCP2 err:%x\n", lcx_irqstatus);
		return -EIO;
	}

	if (lcm_irqstatus & ISPCCP2_LCM_IRQSTATUS_OCPERROR_IRQ) {
		ccp2->error = 1;
		dev_dbg(isp->dev, "CCP2 OCP err:%x\n", lcm_irqstatus);
		ret = -EIO;
	}

	/* Handle queued buffers on frame end interrupts */
	if (lcm_irqstatus & ISPCCP2_LCM_IRQSTATUS_EOF_IRQ)
		ispccp2_isr_buffer(ccp2);

	return ret;
}

/* --------------------------------------------------------------------------
 * ISP ccp2 video device node
 */

/*
 * ispccp2_video_queue - Queue video buffer.
 * @video : Pointer to isp video structure
 * @buffer: Pointer to isp_buffer structure
 * return -EIO or zero on success
 */
static int
ispccp2_video_queue(struct isp_video *video, struct isp_buffer *buffer)
{
	struct isp_ccp2_device *ccp2 = &video->isp->isp_ccp2;

	ispccp2_set_inaddr(ccp2, buffer->isp_addr);
	return 0;
}

static const struct isp_video_operations ispccp2_video_ops = {
	.queue = ispccp2_video_queue,
};

/* -----------------------------------------------------------------------------
 * Media entity operations
 */

/*
 * ispccp2_link_setup - Setup ccp2 connections.
 * @entity : Pointer to media entity structure
 * @local  : Pointer to local pad array
 * @remote : Pointer to remote pad array
 * @flags  : Link flags
 * return -EINVAL on error or zero on success
 */
static int ispccp2_link_setup(struct media_entity *entity,
			const struct media_entity_pad *local,
			const struct media_entity_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct isp_ccp2_device *ccp2 = v4l2_get_subdevdata(sd);

	switch (local->index | (remote->entity->type << 16)) {
	case CCP2_PAD_SINK | (MEDIA_ENTITY_TYPE_NODE << 16):
		/* read from memory */
		if (flags & MEDIA_LINK_FLAG_ACTIVE) {
			if (ccp2->input == CCP2_INPUT_SENSOR)
				return -EBUSY;
			ccp2->input = CCP2_INPUT_MEMORY;
		} else {
			if (ccp2->input == CCP2_INPUT_MEMORY)
				ccp2->input = CCP2_INPUT_NONE;
		}
		break;

	case CCP2_PAD_SINK | (MEDIA_ENTITY_TYPE_SUBDEV << 16):
		/* read from sensor/phy */
		if (flags & MEDIA_LINK_FLAG_ACTIVE)
			ccp2->input = CCP2_INPUT_SENSOR;
		else
			ccp2->input = CCP2_INPUT_NONE;
		break;

	case CCP2_PAD_SOURCE | (MEDIA_ENTITY_TYPE_SUBDEV << 16):
		/* write to video port/ccdc */
		if (flags & MEDIA_LINK_FLAG_ACTIVE)
			ccp2->output = CCP2_OUTPUT_CCDC;
		else
			ccp2->output = CCP2_OUTPUT_NONE;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/* media operations */
static const struct media_entity_operations ccp2_media_ops = {
	.link_setup = ispccp2_link_setup,
};

/*
 * isp_ccp2_init_entities - Initialize ccp2 subdev and media entity.
 * @ccp2 : Pointer to ispccp2 structure
 * return negative error code or zero on success
 */
static int isp_ccp2_init_entities(struct isp_ccp2_device *ccp2)
{
	struct v4l2_subdev *sd = &ccp2->subdev;
	struct media_entity_pad *pads = ccp2->pads;
	struct media_entity *me = &sd->entity;
	int ret;

	ccp2->input = CCP2_INPUT_NONE;
	ccp2->output = CCP2_OUTPUT_NONE;

	v4l2_subdev_init(sd, &ispccp2_sd_ops);
	strlcpy(sd->name, "OMAP3 ISP CCP2", sizeof(sd->name));
	sd->grp_id = 1 << 16;   /* group ID for isp subdevs */
	v4l2_set_subdevdata(sd, ccp2);

	pads[CCP2_PAD_SINK].type = MEDIA_PAD_TYPE_INPUT;
	pads[CCP2_PAD_SOURCE].type = MEDIA_PAD_TYPE_OUTPUT;

	me->ops = &ccp2_media_ops;
	ret = media_entity_init(me, CCP2_PADS_NUM, pads, 0);
	if (ret < 0)
		return ret;

	ccp2->video_in.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	ccp2->video_in.alignment = 32;
	ccp2->video_in.isp = to_isp_device(ccp2);
	ccp2->video_in.ops = &ispccp2_video_ops;
	ccp2->video_in.capture_mem = PAGE_ALIGN(4096 * 4096) * 3;

	ret = isp_video_init(&ccp2->video_in, "CCP2");
	if (ret < 0)
		return ret;

	/* Connect the video node to the ccp2 subdev. */
	ret = media_entity_create_link(&ccp2->video_in.video.entity, 0,
				       &ccp2->subdev.entity, CCP2_PAD_SINK, 0);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * isp_ccp2_unregister_entities - Unregister media entities: subdev
 * @ccp2 - Pointer to ccp2 device
 */
void isp_ccp2_unregister_entities(struct isp_ccp2_device *ccp2)
{
	media_entity_cleanup(&ccp2->subdev.entity);

	v4l2_device_unregister_subdev(&ccp2->subdev);
	isp_video_unregister(&ccp2->video_in);
}

/*
 * ispccp2_register_entities - Register the subdev media entity
 * @ccp2 - Pointer to ccp2 device
 * @vdev - Pointer to v4l device
 * return negative error code or zero on success
 */

int isp_ccp2_register_entities(struct isp_ccp2_device *ccp2,
			       struct v4l2_device *vdev)
{
	int ret;

	/* Register the subdev and video nodes. */
	ret = v4l2_device_register_subdev(vdev, &ccp2->subdev);
	if (ret < 0)
		goto error;

	ret = isp_video_register(&ccp2->video_in, vdev);
	if (ret < 0)
		goto error;

	return 0;

error:
	isp_ccp2_unregister_entities(ccp2);
	return ret;
}

/* -----------------------------------------------------------------------------
 * ISP ccp2 initialisation and cleanup
 */

/*
 * ispccp2_cleanup - CCP2 un-initialization
 * @isp : Pointer to ISP device
 */
void isp_ccp2_cleanup(struct isp_device *isp)
{
}

/*
 * isp_ccp2_init - CCP2 initialization.
 * @isp : Pointer to ISP device
 * return negative error code or zero on success
 */
int isp_ccp2_init(struct isp_device *isp)
{
	struct isp_ccp2_device *ccp2 = &isp->isp_ccp2;
	int ret;

	/* On the OMAP36xx, the CCP2 uses the CSI PHY1 or PHY2, shared with
	 * the CSI2c or CSI2a receivers. The PHY then needs to be explicitly
	 * configured.
	 *
	 * TODO: Don't hardcode the usage of PHY1 (shared with CSI2c).
	 */
	if (isp->revision == ISP_REVISION_15_0)
		ccp2->phy = &isp->isp_csiphy1;

	ret = isp_ccp2_init_entities(ccp2);
	if (ret < 0)
		goto out;

	ispccp2_reset(isp);
out:
	if (ret)
		isp_ccp2_cleanup(isp);

	return ret;
}

