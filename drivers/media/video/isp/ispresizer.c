/*
 * ispresizer.c
 *
 * Driver Library for Resizer module in TI's OMAP3 Camera ISP
 *
 * Copyright (C)2009 Texas Instruments, Inc.
 *
 * Rewritten by: Antti Koskipaa <antti.koskipaa@nokia.com>
 *
 * Based on code by:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Mohit Jalori
 *	Sergio Aguirre <saaguirre@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/mm.h>

#include "isp.h"
#include "ispreg.h"
#include "ispresizer.h"

/*
 * Resizer Constants
 */
#define MIN_RESIZE_VALUE		64
#define MID_RESIZE_VALUE		512
#define MAX_RESIZE_VALUE		1024

#define MIN_IN_WIDTH			32
#define MIN_IN_HEIGHT			32
#define MAX_IN_WIDTH_MEMORY_MODE	4095
#define MAX_IN_WIDTH_ONTHEFLY_MODE_ES1	1280
#define MAX_IN_WIDTH_ONTHEFLY_MODE_ES2	4095
#define MAX_IN_HEIGHT			4095

#define MIN_OUT_WIDTH			16
#define MIN_OUT_HEIGHT			2
#define MAX_OUT_HEIGHT			4095

/*
 * Resizer Use Constraints
 * "TRM ES3.1, table 12-46"
 */
#define MAX_4TAP_OUT_WIDTH_ES1		1280
#define MAX_7TAP_OUT_WIDTH_ES1		640
#define MAX_4TAP_OUT_WIDTH_ES2		3312
#define MAX_7TAP_OUT_WIDTH_ES2		1650
#define MAX_4TAP_OUT_WIDTH_3630		4096
#define MAX_7TAP_OUT_WIDTH_3630		2048

/*
 * Constants for ratio calculation
 */
#define RESIZE_DIVISOR			256
#define DEFAULT_PHASE			1

/*
 * Default (and only) configuration of filter coefficients.
 * 7-tap mode is for scale factors 0.25x to 0.5x.
 * 4-tap mode is for scale factors 0.5x to 4.0x.
 * There shouldn't be any reason to recalculate these, EVER.
 */
static const struct isprsz_coef filter_coefs = {
	/* For 8-phase 4-tap horizontal filter: */
	{
		0x0000, 0x0100, 0x0000, 0x0000,
		0x03FA, 0x00F6, 0x0010, 0x0000,
		0x03F9, 0x00DB, 0x002C, 0x0000,
		0x03FB, 0x00B3, 0x0053, 0x03FF,
		0x03FD, 0x0082, 0x0084, 0x03FD,
		0x03FF, 0x0053, 0x00B3, 0x03FB,
		0x0000, 0x002C, 0x00DB, 0x03F9,
		0x0000, 0x0010, 0x00F6, 0x03FA
	},
	/* For 8-phase 4-tap vertical filter: */
	{
		0x0000, 0x0100, 0x0000, 0x0000,
		0x03FA, 0x00F6, 0x0010, 0x0000,
		0x03F9, 0x00DB, 0x002C, 0x0000,
		0x03FB, 0x00B3, 0x0053, 0x03FF,
		0x03FD, 0x0082, 0x0084, 0x03FD,
		0x03FF, 0x0053, 0x00B3, 0x03FB,
		0x0000, 0x002C, 0x00DB, 0x03F9,
		0x0000, 0x0010, 0x00F6, 0x03FA
	},
	/* For 4-phase 7-tap horizontal filter: */
	#define DUMMY 0
	{
		0x0004, 0x0023, 0x005A, 0x0058, 0x0023, 0x0004, 0x0000, DUMMY,
		0x0002, 0x0018, 0x004d, 0x0060, 0x0031, 0x0008, 0x0000, DUMMY,
		0x0001, 0x000f, 0x003f, 0x0062, 0x003f, 0x000f, 0x0001, DUMMY,
		0x0000, 0x0008, 0x0031, 0x0060, 0x004d, 0x0018, 0x0002, DUMMY
	},
	/* For 4-phase 7-tap vertical filter: */
	{
		0x0004, 0x0023, 0x005A, 0x0058, 0x0023, 0x0004, 0x0000, DUMMY,
		0x0002, 0x0018, 0x004d, 0x0060, 0x0031, 0x0008, 0x0000, DUMMY,
		0x0001, 0x000f, 0x003f, 0x0062, 0x003f, 0x000f, 0x0001, DUMMY,
		0x0000, 0x0008, 0x0031, 0x0060, 0x004d, 0x0018, 0x0002, DUMMY
	}
	/*
	 * The dummy padding is required in 7-tap mode because of how the
	 * registers are arranged physically.
	 */
	#undef DUMMY
};

/* Structure for saving/restoring resizer module registers */
static struct isp_reg isprsz_reg_list[] = {
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_CNT, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_OUT_SIZE, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_IN_START, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_IN_SIZE, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_INADD, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_INOFF, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_OUTADD, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_OUTOFF, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT10, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT32, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT54, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT76, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT98, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT1110, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT1312, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT1514, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT1716, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT1918, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT2120, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT2322, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT2524, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT2726, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT2928, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_HFILT3130, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT10, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT32, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT54, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT76, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT98, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT1110, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT1312, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT1514, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT1716, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT1918, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT2120, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT2322, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT2524, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT2726, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT2928, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_VFILT3130, 0x0000},
	{OMAP3_ISP_IOMEM_RESZ, ISPRSZ_YENH, 0x0000},
	{OMAP3_ISP_IOMEM_SBL, ISPSBL_SDR_REQ_EXP, 0x0000},
	{0, ISP_TOK_TERM, 0x0000}
};

/*
 * __resizer_get_format - helper function for getting resizer format
 * @res   : pointer to resizer private structure
 * @pad   : pad number
 * @which : wanted subdev format
 * return zero
 */
static struct v4l2_mbus_framefmt *
__resizer_get_format(struct isp_res_device *res, unsigned int pad,
		     enum v4l2_subdev_format which)
{
	if (which != V4L2_SUBDEV_FORMAT_PROBE &&
	    which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return NULL;

	if (pad >= RESZ_PADS_NUM)
		return NULL;

	return &res->formats[pad][which];
}

/**
 * ispresizer_set_filters - Set resizer filters
 * @isp_res: Device context.
 * @h_coeff: horizontal coefficient
 * @v_coeff: vertical coefficient
 * Return none
 */
static void ispresizer_set_filters(struct isp_res_device *res,
				   const u16 *h_coeff,
				   const u16 *v_coeff)
{
	struct isp_device *isp = to_isp_device(res);
	u32 startaddr_h, startaddr_v, tmp_h, tmp_v;
	int i;

	startaddr_h = ISPRSZ_HFILT10;
	startaddr_v = ISPRSZ_VFILT10;

	for (i = 0; i < COEFF_CNT; i += 2) {
		tmp_h = h_coeff[i] |
			(h_coeff[i + 1] << ISPRSZ_HFILT_COEF1_SHIFT);
		tmp_v = v_coeff[i] |
			(v_coeff[i + 1] << ISPRSZ_VFILT_COEF1_SHIFT);
		isp_reg_writel(isp, tmp_h, OMAP3_ISP_IOMEM_RESZ, startaddr_h);
		isp_reg_writel(isp, tmp_v, OMAP3_ISP_IOMEM_RESZ, startaddr_v);
		startaddr_h += 4;
		startaddr_v += 4;
	}
}

/**
 * ispresizer_set_bilinear - Chrominance horizontal algorithm select
 * @isp_res: Device context.
 * @type: Filtering interpolation type.
 *
 * Filtering that is same as luminance processing is
 * intended only for downsampling, and bilinear interpolation
 * is intended only for upsampling.
 */
static void ispresizer_set_bilinear(struct isp_res_device *res,
				    enum resizer_chroma_algo type)
{
	struct isp_device *isp = to_isp_device(res);

	if (type == RSZ_BILINEAR)
		isp_reg_or(isp, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_CNT,
			   ISPRSZ_CNT_CBILIN);
	else
		isp_reg_and(isp, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_CNT,
			    ~ISPRSZ_CNT_CBILIN);
}

/**
 * ispresizer_set_ycpos - Luminance and chrominance order
 * @isp_res: Device context.
 * @order: order type.
 */
static void ispresizer_set_ycpos(struct isp_res_device *res,
				 enum v4l2_mbus_pixelcode pixelcode)
{
	struct isp_device *isp = to_isp_device(res);

	switch (pixelcode) {
	case V4L2_MBUS_FMT_YUYV16_1X16:
		isp_reg_or(isp, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_CNT,
			   ISPRSZ_CNT_YCPOS);
		break;
	case V4L2_MBUS_FMT_UYVY16_1X16:
		isp_reg_and(isp, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_CNT,
			    ~ISPRSZ_CNT_YCPOS);
		break;
	default:
		return;
	}
}

/**
 * ispresizer_set_phase - Setup horizontal and vertical starting phase
 * @isp_res: Device context.
 * @h_phase: horizontal phase parameters.
 * @v_phase: vertical phase parameters.
 *
 * Horizontal and vertical phase range is 0 to 7
 */
static void ispresizer_set_phase(struct isp_res_device *res, u32 h_phase,
				 u32 v_phase)
{
	struct isp_device *isp = to_isp_device(res);
	u32 rgval = 0;

	rgval = isp_reg_readl(isp, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_CNT) &
	      ~(ISPRSZ_CNT_HSTPH_MASK | ISPRSZ_CNT_VSTPH_MASK);
	rgval |= (h_phase << ISPRSZ_CNT_HSTPH_SHIFT) & ISPRSZ_CNT_HSTPH_MASK;
	rgval |= (v_phase << ISPRSZ_CNT_VSTPH_SHIFT) & ISPRSZ_CNT_VSTPH_MASK;

	isp_reg_writel(isp, rgval, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_CNT);
}

/**
 * ispresizer_set_luma - Setup luminance enhancer parameters
 * @isp_res: Device context.
 * @luma: Structure for luminance enhancer parameters.
 *
 * Algorithm select:
 *  0x0: Disable
 *  0x1: [-1  2 -1]/2 high-pass filter
 *  0x2: [-1 -2  6 -2 -1]/4 high-pass filter
 *
 * Maximum gain:
 *  The data is coded in U4Q4 representation.
 *
 * Slope:
 *  The data is coded in U4Q4 representation.
 *
 * Coring offset:
 *  The data is coded in U8Q0 representation.
 *
 * The new luminance value is computed as:
 *  Y += HPF(Y) x max(GAIN, (HPF(Y) - CORE) x SLOP + 8) >> 4.
 */
static void ispresizer_set_luma(struct isp_res_device *res,
				struct resizer_luma_yenh *luma)
{
	struct isp_device *isp = to_isp_device(res);
	u32 rgval = 0;

	rgval  = (luma->algo << ISPRSZ_YENH_ALGO_SHIFT)
		  & ISPRSZ_YENH_ALGO_MASK;
	rgval |= (luma->gain << ISPRSZ_YENH_GAIN_SHIFT)
		  & ISPRSZ_YENH_GAIN_MASK;
	rgval |= (luma->slope << ISPRSZ_YENH_SLOP_SHIFT)
		  & ISPRSZ_YENH_SLOP_MASK;
	rgval |= (luma->core << ISPRSZ_YENH_CORE_SHIFT)
		  & ISPRSZ_YENH_CORE_MASK;

	isp_reg_writel(isp, rgval, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_YENH);
}

/**
 * ispresizer_set_source - Input source select
 * @isp_res: Device context.
 * @source: Input source type
 *
 * If this field is set to RSZ_OTFLY_YUV, the resizer input is fed from
 * Preview/CCDC engine, otherwise from memory.
 */
static void ispresizer_set_source(struct isp_res_device *res,
				  enum resizer_input source)
{
	struct isp_device *isp = to_isp_device(res);

	if (source != RSZ_OTFLY_YUV)
		isp_reg_or(isp, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_CNT,
			   ISPRSZ_CNT_INPSRC);
	else
		isp_reg_and(isp, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_CNT,
			    ~ISPRSZ_CNT_INPSRC);
}

/**
 * ispresizer_set_ratio - Setup horizontal and vertical resizing value
 * @isp_res: Device context.
 * @ratio: Structure for ratio parameters.
 *
 * Resizing range from 64 to 1024
 */
static void ispresizer_set_ratio(struct isp_res_device *res,
				 const struct resizer_ratio *ratio)
{
	struct isp_device *isp = to_isp_device(res);
	const u16 *h_filter, *v_filter;
	u32 rgval = 0;

	rgval = isp_reg_readl(isp, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_CNT) &
			      ~(ISPRSZ_CNT_HRSZ_MASK | ISPRSZ_CNT_VRSZ_MASK);
	rgval |= ((ratio->horz - 1) << ISPRSZ_CNT_HRSZ_SHIFT)
		  & ISPRSZ_CNT_HRSZ_MASK;
	rgval |= ((ratio->vert - 1) << ISPRSZ_CNT_VRSZ_SHIFT)
		  & ISPRSZ_CNT_VRSZ_MASK;
	isp_reg_writel(isp, rgval, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_CNT);

	/* prepare horizontal filter coefficients */
	if (ratio->horz > MID_RESIZE_VALUE)
		h_filter = &filter_coefs.h_filter_coef_7tap[0];
	else
		h_filter = &filter_coefs.h_filter_coef_4tap[0];

	/* prepare vertical filter coefficients */
	if (ratio->vert > MID_RESIZE_VALUE)
		v_filter = &filter_coefs.v_filter_coef_7tap[0];
	else
		v_filter = &filter_coefs.v_filter_coef_4tap[0];

	ispresizer_set_filters(res, h_filter, v_filter);
}

/**
 * ispresizer_set_dst_size - Setup the output height and width
 * @isp_res: Device context.
 * @width: Output width.
 * @height: Output height.
 *
 * Width :
 *  The value must be EVEN.
 *
 * Height:
 *  The number of bytes written to SDRAM must be
 *  a multiple of 16-bytes if the vertical resizing factor
 *  is greater than 1x (upsizing)
 */
static void ispresizer_set_output_size(struct isp_res_device *res,
				       u32 width, u32 height)
{
	struct isp_device *isp = to_isp_device(res);
	u32 rgval = 0;

	dev_dbg(isp->dev, "Output size[w/h]: %dx%d\n", width, height);
	rgval  = (width << ISPRSZ_OUT_SIZE_HORZ_SHIFT)
		 & ISPRSZ_OUT_SIZE_HORZ_MASK;
	rgval |= (height << ISPRSZ_OUT_SIZE_VERT_SHIFT)
		 & ISPRSZ_OUT_SIZE_VERT_MASK;
	isp_reg_writel(isp, rgval, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_OUT_SIZE);
}

/**
 * ispresizer_set_output_offset - Setup memory offset for the output lines.
 * @isp_res: Device context.
 * @offset: Memory offset.
 *
 * The 5 LSBs are forced to be zeros by the hardware to align on a 32-byte
 * boundary; the 5 LSBs are read-only. For optimal use of SDRAM bandwidth,
 * the SDRAM line offset must be set on a 256-byte boundary
 */
static void ispresizer_set_output_offset(struct isp_res_device *res,
					 u32 offset)
{
	struct isp_device *isp = to_isp_device(res);

	isp_reg_writel(isp, offset, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_OUTOFF);
}

/**
 * ispresizer_set_start - Setup vertical and horizontal start position
 * @isp_res: Device context.
 * @left: Horizontal start position.
 * @top: Vertical start position.
 *
 * Vertical start line:
 *  This field makes sense only when the resizer obtains its input
 *  from the preview engine/CCDC
 *
 * Horizontal start pixel:
 *  Pixels are coded on 16 bits for YUV and 8 bits for color separate data.
 *  When the resizer gets its input from SDRAM, this field must be set
 *  to <= 15 for YUV 16-bit data and <= 31 for 8-bit color separate data
 */
static void ispresizer_set_start(struct isp_res_device *res, u32 left,
				 u32 top)
{
	struct isp_device *isp = to_isp_device(res);
	u32 rgval = 0;

	rgval = (left << ISPRSZ_IN_START_HORZ_ST_SHIFT)
		& ISPRSZ_IN_START_HORZ_ST_MASK;
	rgval |= (top << ISPRSZ_IN_START_VERT_ST_SHIFT)
		 & ISPRSZ_IN_START_VERT_ST_MASK;

	isp_reg_writel(isp, rgval, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_IN_START);
}

/**
 * ispresizer_set_input_size - Setup the input size
 * @isp_res: Device context.
 * @width: The range is 0 to 4095 pixels
 * @height: The range is 0 to 4095 lines
 */
static void ispresizer_set_input_size(struct isp_res_device *res,
				      u32 width, u32 height)
{
	struct isp_device *isp = to_isp_device(res);
	u32 rgval = 0;

	dev_dbg(isp->dev, "Input size[w/h]: %dx%d\t", width, height);

	rgval = (width << ISPRSZ_IN_SIZE_HORZ_SHIFT)
		& ISPRSZ_IN_SIZE_HORZ_MASK;
	rgval |= (height << ISPRSZ_IN_SIZE_VERT_SHIFT)
		 & ISPRSZ_IN_SIZE_VERT_MASK;

	isp_reg_writel(isp, rgval, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_IN_SIZE);
}

/**
 * ispresizer_set_src_offs - Setup the memory offset for the input lines
 * @isp_res: Device context.
 * @offset: Memory offset.
 *
 * The 5 LSBs are forced to be zeros by the hardware to align on a 32-byte
 * boundary; the 5 LSBs are read-only. This field must be programmed to be
 * 0x0 if the resizer input is from preview engine/CCDC.
 */
static void ispresizer_set_input_offset(struct isp_res_device *res,
					u32 offset)
{
	struct isp_device *isp = to_isp_device(res);

	isp_reg_writel(isp, offset, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_INOFF);
}

/**
 * ispresizer_set_intype - Input type select
 * @isp_res: Device context.
 * @type: Pixel format type.
 */
static void ispresizer_set_intype(struct isp_res_device *res,
				  enum resizer_colors_type type)
{
	struct isp_device *isp = to_isp_device(res);

	if (type == RSZ_COLOR8)
		isp_reg_or(isp, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_CNT,
			   ISPRSZ_CNT_INPTYP);
	else
		isp_reg_and(isp, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_CNT,
			    ~ISPRSZ_CNT_INPTYP);
}

/**
 * __ispresizer_set_inaddr - Helper function for set input address
 * @res : pointer to resizer private data structure
 * @addr: input address
 * return none
 */
static void __ispresizer_set_inaddr(struct isp_res_device *res, u32 addr)
{
	struct isp_device *isp = to_isp_device(res);

	isp_reg_writel(isp, addr, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_INADD);
}

/**
 * ispresizer_adjust_bandwidth - Reduces read bandwidth when scaling up.
 * Otherwise there will be SBL overflows.
 *
 * The ISP read speed is 256.0 / max(256, 1024 * ISPSBL_SDR_REQ_EXP). This
 * formula is correct, no matter what the TRM says. Thus, the first
 * step to use is 0.25 (REQ_EXP=1).
 *
 * Ratios:
 * 0 = 1.0
 * 1 = 0.25
 * 2 = 0.125
 * 3 = 0.083333...
 * 4 = 0.0625
 * 5 = 0.05 and so on...
 *
 * TRM says that read bandwidth should be no more than 83MB/s, half
 * of the maximum of 166MB/s.
 *
 * HOWEVER, the read speed must be chosen so that the resizer always
 * has time to process the frame before the next frame comes in.
 * Failure to do so will result in a pile-up and endless "resizer busy!"
 * messages.
 *
 * Zoom ratio must not exceed 4.0. This is checked in
 * ispresizer_check_crop_boundaries().
 **/
static void ispresizer_adjust_bandwidth(struct isp_res_device *res,
					int input_width, int input_height,
					int output_width, int output_height)
{
	struct isp_device *isp = to_isp_device(res);

	/* Table for dividers. This allows hand tuning. */
	static const unsigned char area_to_divider[] = {
		0, 0, 0, 1, 1, 1, 2, 2, 2, 3, 3, 4, 4, 5, 5, 5
	     /* 1........2...........3.......................4 Zoom level */
	};
	unsigned int input_area = input_width * input_height;
	unsigned int output_area = output_width * output_height;

	if (input_area < output_area && input_area > 0) {
		u32 val = area_to_divider[output_area / input_area - 1];
		dev_dbg(isp->dev, "%s: area factor = %i, val = %i\n",
				__func__, output_area / input_area, val);
		isp_reg_writel(isp, val << ISPSBL_SDR_REQ_RSZ_EXP_SHIFT,
			       OMAP3_ISP_IOMEM_SBL, ISPSBL_SDR_REQ_EXP);
	} else {
		/* Required input bandwidth greater than output, no limit. */
		dev_dbg(isp->dev, "%s: resetting\n", __func__);
		isp_reg_writel(isp, 0, OMAP3_ISP_IOMEM_SBL,
			       ISPSBL_SDR_REQ_EXP);
	}
}

/**
 * ispresizer_busy - Checks if ISP resizer is busy.
 *
 * Returns busy field from ISPRSZ_PCR register.
 **/
int ispresizer_busy(struct isp_res_device *res)
{
	struct isp_device *isp = to_isp_device(res);

	return isp_reg_readl(isp, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_PCR) &
			     ISPRSZ_PCR_BUSY;
}

/**
 * ispresizer_set_inaddr - Sets the memory address of the input frame.
 * @addr: 32bit memory address aligned on 32byte boundary.
 *
 * Returns 0 if successful, or -EINVAL if address is not 32 bits aligned.
 **/
static int ispresizer_set_inaddr(struct isp_res_device *res, u32 addr)
{
	struct isp_device *isp = to_isp_device(res);

	dev_dbg(isp->dev, "%s: addr = 0x%08x\n", __func__, addr);

	res->addr_base = addr;

	/* This will handle crop settings in stream off state */
	if (res->crop_offset)
		addr += res->crop_offset & ~0xf;

	__ispresizer_set_inaddr(res, addr);

	return 0;
}

/*
 * Configures the memory address to which the output frame is written.
 * @addr: 32bit memory address aligned on 32byte boundary.
 * Note: For SBL efficiency reasons the address should be on a 256-byte
 * boundary.
 */
static void ispresizer_set_outaddr(struct isp_res_device *res, u32 addr)
{
	struct isp_device *isp = to_isp_device(res);

	/*
	 * Set output address. This needs to be in its own function
	 * because it changes often.
	 */
	isp_reg_writel(isp, addr << ISPRSZ_SDR_OUTADD_ADDR_SHIFT,
		       OMAP3_ISP_IOMEM_RESZ, ISPRSZ_SDR_OUTADD);
}

/**
 * ispresizer_save_context - Saves the values of the resizer module registers.
 **/
void ispresizer_save_context(struct isp_device *isp)
{
	dev_dbg(isp->dev, "Saving context\n");
	isp_save_context(isp, isprsz_reg_list);
}

/**
 * ispresizer_restore_context - Restores resizer module register values.
 **/
void ispresizer_restore_context(struct isp_device *isp)
{
	dev_dbg(isp->dev, "Restoring context\n");
	isp_restore_context(isp, isprsz_reg_list);
}

/*
 * ispresizer_calc_ratios - Helper function for calculate resizer ratios
 * @res: pointer to resizer private data structure
 * @input: input frame size
 * @output: output frame size
 * @ratio : return calculated ratios
 * return none
 *
 * The resizer uses a polyphase sample rate converter. The upsampling filter
 * has a fixed number of phases that depend on the resizing ratio. As the ratio
 * computation depends on the number of phases, we need to compute a first
 * approximation and then refine it.
 *
 * The input/output/ratio relationship is given by the OMAP34xx TRM:
 *
 * - 8-phase, 4-tap mode (RSZ = 64 ~ 512)
 *	iw = (32 * sph + (ow - 1) * hrsz + 16) >> 8 + 7
 *	ih = (32 * spv + (oh - 1) * vrsz + 16) >> 8 + 4
 * - 4-phase, 7-tap mode (RSZ = 513 ~ 1024)
 *	iw = (64 * sph + (ow - 1) * hrsz + 32) >> 8 + 7
 *	ih = (64 * spv + (oh - 1) * vrsz + 32) >> 8 + 7
 *
 * iw and ih are the input width and height after cropping. Those equations need
 * to be satisfied exactly for the resizer to work correctly.
 *
 * Reverting the equations, we can compute the resizing ratios with
 *
 * - 8-phase, 4-tap mode
 *	hrsz = ((iw - 7) * 256 - 16 - 32 * sph) / (ow - 1)
 *	vrsz = ((ih - 4) * 256 - 16 - 32 * spv) / (oh - 1)
 * - 4-phase, 7-tap mode
 *	hrsz = ((iw - 7) * 256 - 32 - 64 * sph) / (ow - 1)
 *	vrsz = ((ih - 7) * 256 - 32 - 64 * spv) / (oh - 1)
 *
 * The ratios are integer values, and must be rounded down to ensure that the
 * cropped input size is not bigger than the uncropped input size. As the ratio
 * in 7-tap mode is always smaller than the ratio in 4-tap mode, we can use the
 * 7-tap mode equations to compute a ratio approximation.
 *
 * We first clamp the output size according to the hardware capabilitie to avoid
 * auto-cropping the input more than required to satisfy the TRM equations. The
 * worst case is 7-tap mode, which will lead to the smallest output width. We
 * can thus compute the minimum and maximum output sizes with
 *
 * - 4-phase, 7-tap mode
 *	min ow = ((iw - 7) * 256 - 32 - 64 * sph) / 1024 + 1
 *	min oh = ((ih - 7) * 256 - 32 - 64 * spv) / 1024 + 1
 *	max ow = ((iw - 7) * 256 - 32 - 64 * sph) / 64 + 1
 *	max oh = ((ih - 7) * 256 - 32 - 64 * spv) / 64 + 1
 *
 * We then compute and clamp the ratios (x1/4 ~ x4). Clamping the output size to
 * the maximum value guarantees that the ratio value will never be smaller than
 * the minimum, but it could still slightly exceed the maximum. Clamping the
 * ratio will thus result in a resizing factor slightly larger than the
 * requested value.
 *
 * To accomodate that, and make sure the TRM equations are satisfied exactly, we
 * compute the input crop rectangle as the last step.
 *
 * As if the situation wasn't complex enough, the maximum output width depends
 * on the vertical resizing ratio.  Fortunately, the output height doesn't
 * depend on the horizontal resizing ratio. We can then start by computing the
 * output height and the vertical ratio, and then move to computing the output
 * width and the horizontal ratio.
 */
static void ispresizer_calc_ratios(struct isp_res_device *res,
				   struct v4l2_rect *input,
				   struct v4l2_mbus_framefmt *output,
				   struct resizer_ratio *ratio)
{
	struct isp_device *isp = to_isp_device(res);
	const unsigned int spv = DEFAULT_PHASE;
	const unsigned int sph = DEFAULT_PHASE;
	unsigned int upscaled_width;
	unsigned int upscaled_height;
	unsigned int min_width;
	unsigned int min_height;
	unsigned int max_width;
	unsigned int max_height;
	unsigned int width_alignment;

	/*
	 * Clamp the output height based on the hardware capabilities and
	 * compute the vertical resizing ratio.
	 */
	min_height = ((input->height - 7) * 256 - 32 - 64 * spv) / 1024 + 1;
	min_height = max_t(unsigned int, min_height, MIN_OUT_HEIGHT);
	max_height = ((input->height - 7) * 256 - 32 - 64 * spv) / 64 + 1;
	max_height = min_t(unsigned int, max_height, MAX_OUT_HEIGHT);
	output->height = clamp(output->height, min_height, max_height);

	ratio->vert = ((input->height - 7) * 256 - 32 - 64 * spv)
		    / (output->height - 1);
	ratio->vert = clamp_t(unsigned int, ratio->vert,
			      MIN_RESIZE_VALUE, MAX_RESIZE_VALUE);

	if (ratio->vert <= MID_RESIZE_VALUE) {
		upscaled_height = (output->height - 1) * ratio->vert
				+ 32 * spv + 16;
		input->height = (upscaled_height >> 8) + 4;
	} else {
		upscaled_height = (output->height - 1) * ratio->vert
				+ 64 * spv + 32;
		input->height = (upscaled_height >> 8) + 7;
	}

	/*
	 * Compute the minimum and maximum output widths based on the hardware
	 * capabilities. The maximum depends on the vertical resizing ratio.
	 */
	min_width = ((input->width - 7) * 256 - 32 - 64 * sph) / 1024 + 1;
	min_width = max_t(unsigned int, min_width, MIN_OUT_WIDTH);

	if (ratio->vert <= MID_RESIZE_VALUE) {
		switch (isp->revision) {
		case ISP_REVISION_1_0:
			max_width = MAX_4TAP_OUT_WIDTH_ES1;
			break;

		case ISP_REVISION_2_0:
		default:
			max_width = MAX_4TAP_OUT_WIDTH_ES2;
			break;

		case ISP_REVISION_15_0:
			max_width = MAX_4TAP_OUT_WIDTH_3630;
			break;
		}
	} else {
		switch (isp->revision) {
		case ISP_REVISION_1_0:
			max_width = MAX_7TAP_OUT_WIDTH_ES1;
			break;

		case ISP_REVISION_2_0:
		default:
			max_width = MAX_7TAP_OUT_WIDTH_ES2;
			break;

		case ISP_REVISION_15_0:
			max_width = MAX_7TAP_OUT_WIDTH_3630;
			break;
		}
	}
	max_width = min(((input->width - 7) * 256 - 32 - 64 * sph) / 64 + 1,
			max_width);

	/*
	 * The output width must be even, and must be a multiple of 16 bytes
	 * when upscaling vertically. Clamp the output width to the valid range.
	 * Take the alignment into account (the maximum width in 7-tap mode on
	 * ES2 isn't a multiple of 8) and align the result up to make sure it
	 * won't be smaller than the minimum.
	 */
	width_alignment = ratio->vert < 256 ? 8 : 2;
	output->width = clamp(output->width, min_width,
			      max_width & ~(width_alignment - 1));
	output->width = ALIGN(output->width, width_alignment);

	ratio->horz = ((input->width - 7) * 256 - 32 - 64 * sph)
		    / (output->width - 1);
	ratio->horz = clamp_t(unsigned int, ratio->horz,
			      MIN_RESIZE_VALUE, MAX_RESIZE_VALUE);

	if (ratio->horz <= MID_RESIZE_VALUE) {
		upscaled_width = (output->width - 1) * ratio->horz
			       + 32 * spv + 16;
		input->width = (upscaled_width >> 8) + 7;
	} else {
		upscaled_width = (output->width - 1) * ratio->horz
			       + 64 * spv + 32;
		input->width = (upscaled_width >> 8) + 7;
	}
}

/**
 * ispresizer_set_crop_params - Setup hardware with cropping parameters
 * @res : ispresizer private structure
 * @crop_rect : current crop rectangle
 * @ratio : resizer ratios
 * return none
 */
static void ispresizer_set_crop_params(struct isp_res_device *res,
				       const struct v4l2_rect *crop_rect,
				       const struct v4l2_mbus_framefmt *output,
				       const struct resizer_ratio *ratio)
{
	ispresizer_set_ratio(res, ratio);

	/* Set chrominance horizontal algorithm */
	if (ratio->horz >= RESIZE_DIVISOR)
		ispresizer_set_bilinear(res, RSZ_THE_SAME);
	else
		ispresizer_set_bilinear(res, RSZ_BILINEAR);

	if (res->input == RESIZER_INPUT_MEMORY) {
		ispresizer_adjust_bandwidth(res, crop_rect->width,
					    crop_rect->height,
					    output->width, output->height);

		/* Calculate additional offset for crop */
		res->crop_offset = (crop_rect->top * crop_rect->width +
				    crop_rect->left) * ISP_BYTES_PER_PIXEL;
		/* Write the rest for pixel offset, line offset must be 0 */
		ispresizer_set_start(res, res->crop_offset & 0xf, 0);

		/*
		 * Set start (read) address for cropping.
		 * Address = buffer +
		 *           (top * uncropped_width + left) * bytesperpixel
		 * Lower 16 bits of the resulting offset are put in IN_START
		 * register.
		 */
		__ispresizer_set_inaddr(res,
				res->addr_base + (res->crop_offset & ~0xf));
	} else {
		/*
		 * Set vertical start line and horizontal starting pixel.
		 * If the input is from CCDC/PREV, horizontal start field is
		 * in bytes (twice number of pixels).
		 */
		ispresizer_set_start(res,
				     crop_rect->left * ISP_BYTES_PER_PIXEL,
				     crop_rect->top);
	}

	/* Set the input size */
	ispresizer_set_input_size(res, crop_rect->width, crop_rect->height);
}

static void resizer_enable_oneshot(struct isp_res_device *res)
{
	struct isp_device *isp = to_isp_device(res);

	isp_reg_or(isp, OMAP3_ISP_IOMEM_RESZ, ISPRSZ_PCR,
		   ISPRSZ_PCR_ENABLE | ISPRSZ_PCR_ONESHOT);
}

void ispresizer_isr_frame_sync(struct isp_res_device *res)
{
	if (res->underrun && res->state == ISP_PIPELINE_STREAM_CONTINUOUS) {
		resizer_enable_oneshot(res);
		res->underrun = 0;
	}
}

static void ispresizer_isr_buffer(struct isp_res_device *res)
{
	struct isp_video *video_out = res->video;
	struct isp_buffer *buffer;
	int restart = 0;

	if (res->state == ISP_PIPELINE_STREAM_STOPPED)
		return;

	/* Complete the output buffer and, if reading from memory, the input
	 * buffer.
	 */
	buffer = isp_video_buffer_next(video_out, res->error);
	if (buffer != NULL) {
		ispresizer_set_outaddr(res, buffer->isp_addr);
		restart = 1;
	}

	if (res->input == RESIZER_INPUT_MEMORY) {
		buffer = isp_video_buffer_next(&res->video_in, 0);
		if (buffer != NULL)
			ispresizer_set_inaddr(res, buffer->isp_addr);
	}

	if (res->state == ISP_PIPELINE_STREAM_SINGLESHOT) {
		if (isp_pipeline_ready(video_out->pipe))
			isp_pipeline_set_stream(video_out->isp, video_out,
						ISP_PIPELINE_STREAM_SINGLESHOT);
	} else {
		/* If an underrun occurs, the video queue operation handler will
		 * restart the resizer. Otherwise restart it immediately.
		 */
		if (restart)
			resizer_enable_oneshot(res);
	}

	res->error = 0;
}

/*
 * ispresizer_isr - ISP resizer interrupt handler
 *
 * Manage the resizer video buffers and configure shadowed and busy-locked
 * registers.
 */
void ispresizer_isr(struct isp_res_device *res)
{
	struct v4l2_mbus_framefmt *format;

	if (res->applycrop) {
		format = __resizer_get_format(res, RESZ_PAD_SOURCE,
					      V4L2_SUBDEV_FORMAT_ACTIVE);
		ispresizer_set_crop_params(res, &res->crop.c, format,
					   &res->ratio);
		res->applycrop = 0;
	}

	ispresizer_isr_buffer(res);
}

/* -----------------------------------------------------------------------------
 * ISP video operations
 */

static int resizer_video_queue(struct isp_video *video,
			       struct isp_buffer *buffer)
{
	struct isp_res_device *res = &video->isp->isp_res;

	if (video->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		ispresizer_set_inaddr(res, buffer->isp_addr);

	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		ispresizer_set_outaddr(res, buffer->isp_addr);

		/* We now have a buffer queued on the output. Despite what the
		 * TRM says, the resizer can't be restarted immediately.
		 * Enabling it in one shot mode in the middle of a frame (or at
		 * least asynchronously to the frame) results in the output
		 * being shifted randomly left/right and up/down, as if the
		 * hardware didn't synchronize itself to the beginning of the
		 * frame correctly.
		 *
		 * Restart the resizer on the next sync interrupt if running in
		 * continuous mode.
		 */
		if (res->state == ISP_PIPELINE_STREAM_CONTINUOUS)
			res->underrun = 1;
	}

	return 0;
}

static const struct isp_video_operations resizer_video_ops = {
	.queue = resizer_video_queue,
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev operations
 */

/**
 * resizer_s_power - Handle set power subdev method
 * @sd: pointer to v4l2 subdev structure
 * @on: power on/off
 * return -EINVAL or zero on success
 */
static int resizer_s_power(struct v4l2_subdev *sd, int on)
{
	struct isp_res_device *res = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(res);

	if (on) {
		if (!isp_get(isp))
			return -EBUSY;

		isp_reg_or(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL,
			   ISPCTRL_RSZ_CLK_EN);
	} else {
		isp_reg_and(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL,
			    ~ISPCTRL_RSZ_CLK_EN);
		isp_put(isp);
	}

	return 0;
}

/*
 * resizer_s_stream - Enable/Disable streaming on resizer subdev
 * @sd: ISP resizer V4L2 subdev
 * @enable: 1 == Enable, 0 == Disable
 *
 * The resizer hardware can't be enabled without a memory buffer to write to.
 * As the s_stream operation is called in response to a STREAMON call without
 * any buffer queued yet, just update the state field and return immediately.
 * The resizer will be enabled in resizer_video_queue().
 */
static int resizer_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct isp_res_device *res = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(res);

	res->underrun = 0;

	switch (enable) {
	case ISP_PIPELINE_STREAM_CONTINUOUS:
		isp_sbl_enable(isp, OMAP3_ISP_SBL_RESIZER_WRITE);
		break;

	case ISP_PIPELINE_STREAM_SINGLESHOT:
		if (res->input == RESIZER_INPUT_MEMORY)
			isp_sbl_enable(isp, OMAP3_ISP_SBL_RESIZER_READ);
		isp_sbl_enable(isp, OMAP3_ISP_SBL_RESIZER_WRITE);

		resizer_enable_oneshot(res);
		break;

	case ISP_PIPELINE_STREAM_STOPPED:
		isp_sbl_disable(isp, OMAP3_ISP_SBL_RESIZER_READ |
				OMAP3_ISP_SBL_RESIZER_WRITE);
		break;
	}

	res->state = enable;
	return 0;
}

/* resizer pixel formats */
const static unsigned int resz_fmts[] = {
	V4L2_MBUS_FMT_UYVY16_1X16,
	V4L2_MBUS_FMT_YUYV16_1X16,
};

/**
 * resizer_cropcap - handle cropcap subdev operation
 * @sd : pointer to v4l2 subdev structure
 * @cc : pointer to cropcap structure
 * return zero
 */
static int resizer_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *cc)
{
	struct isp_res_device *res = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;

	format = __resizer_get_format(res, RESZ_PAD_SINK,
				      V4L2_SUBDEV_FORMAT_ACTIVE);
	cc->defrect.width = format->width;
	cc->defrect.height = format->height;
	cc->defrect.left = 0;
	cc->defrect.top = 0;
	cc->bounds = cc->defrect;
	cc->pixelaspect.numerator = 1;
	cc->pixelaspect.denominator = 1;
	return 0;
}

/**
 * resizer_g_crop - handle get crop subdev operation
 * @sd : pointer to v4l2 subdev structure
 * @crop : pointer to crop structure
 * return zero
 */
static int resizer_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
	struct isp_res_device *res = v4l2_get_subdevdata(sd);

	*crop = res->crop;
	return 0;
}

/**
 * resizer_s_crop - handle set crop subdev operation
 * @sd : pointer to v4l2 subdev structure
 * @crop : pointer to crop structure
 * return -EINVAL or zero when succeed
 */
static int resizer_s_crop(struct v4l2_subdev *sd, struct v4l2_crop *crop)
{
	struct isp_res_device *res = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(res);
	struct v4l2_rect *crop_rect = &crop->c;
	struct v4l2_mbus_framefmt *format_sink, *format_source;

	dev_dbg(isp->dev, "%s: L=%d,T=%d,W=%d,H=%d\n", __func__,
		crop_rect->left, crop_rect->top, crop_rect->width,
		crop_rect->height);

	format_sink = __resizer_get_format(res, RESZ_PAD_SINK,
					   V4L2_SUBDEV_FORMAT_ACTIVE);
	format_source = __resizer_get_format(res, RESZ_PAD_SOURCE,
					     V4L2_SUBDEV_FORMAT_ACTIVE);

	dev_dbg(isp->dev, "%s: input=%dx%d, output=%dx%d\n", __func__,
		format_sink->width, format_sink->height,
		format_source->width, format_source->height);

	/* Crop can not go beyond of the input rectangle */
	crop_rect->left = clamp_t(u32, crop_rect->left, 0,
				  format_sink->width - MIN_IN_WIDTH);
	crop_rect->width = clamp_t(u32, crop_rect->width, MIN_IN_WIDTH,
				   format_sink->width - crop_rect->left);
	crop_rect->top = clamp_t(u32, crop_rect->top, 0,
				 format_sink->height - MIN_IN_HEIGHT);
	crop_rect->height = clamp_t(u32, crop_rect->height, MIN_IN_HEIGHT,
				    format_sink->height - crop_rect->top);

	ispresizer_calc_ratios(res, crop_rect, format_source, &res->ratio);

	res->crop = *crop;

	/*
	 * s_crop can be called while streaming is on. In this case
	 * the crop values will be set in the next IRQ.
	 */
	if (res->state != ISP_PIPELINE_STREAM_STOPPED) {
		res->applycrop = 1;
		return 0;
	}

	ispresizer_set_crop_params(res, crop_rect, format_source, &res->ratio);

	return 0;
}

/*
 * resizer_enum_mbus_code - Handle pixel format enumeration
 * @sd     : pointer to v4l2 subdev structure
 * @code   : pointer to v4l2_subdev_pad_mbus_code_enum structure
 * return -EINVAL or zero on success
 */
static int resizer_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_mbus_code_enum *code)
{
	if (code->pad >= RESZ_PADS_NUM ||
	    code->index >= ARRAY_SIZE(resz_fmts))
		return -EINVAL;

	code->code = resz_fmts[code->index];
	return 0;
}

/**
 * resizer_get_format - Handle get format by pads subdev method
 * @sd    : pointer to v4l2 subdev structure
 * @pad   : pad num
 * @fmt   : pointer to v4l2 format structure
 * @which : wanted subdev format
 * return -EINVAL or zero on sucess
 */
static int resizer_get_format(struct v4l2_subdev *sd, unsigned int pad,
			      struct v4l2_mbus_framefmt *fmt,
			      enum v4l2_subdev_format which)
{
	struct isp_res_device *res = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format =
		__resizer_get_format(res, pad, which);

	if (format == NULL)
		return -EINVAL;

	*fmt = *format;
	return 0;
}

/**
 * resizer_try_format - Handle try format by pad subdev method
 * @sd    : pointer to v4l2 subdev structure
 * @pad   : pad num
 * @fmt   : pointer to v4l2 format structure
 * @which : wanted subdev format
 * return -EINVAL or zero on success
 */
static int resizer_try_format(struct v4l2_subdev *sd, unsigned int pad,
			      struct v4l2_mbus_framefmt *fmt,
			      enum v4l2_subdev_format which)
{
	struct isp_res_device *res = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(res);
	unsigned int max_in_width;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect crop_rect;
	struct resizer_ratio ratio;

	switch (pad) {
	case RESZ_PAD_SINK:
		if (fmt->code != V4L2_MBUS_FMT_YUYV16_1X16 &&
		    fmt->code != V4L2_MBUS_FMT_UYVY16_1X16)
			fmt->code = V4L2_MBUS_FMT_YUYV16_1X16;

		if (res->input == RESIZER_INPUT_MEMORY) {
			max_in_width = MAX_IN_WIDTH_MEMORY_MODE;
		} else {
			if (isp->revision == ISP_REVISION_1_0)
				max_in_width = MAX_IN_WIDTH_ONTHEFLY_MODE_ES1;
			else
				max_in_width = MAX_IN_WIDTH_ONTHEFLY_MODE_ES2;
		}

		fmt->width = clamp_t(u32, fmt->width, MIN_IN_WIDTH,
				     max_in_width);
		fmt->height = clamp_t(u32, fmt->height, MIN_IN_HEIGHT,
				      MAX_IN_HEIGHT);
		break;

	case RESZ_PAD_SOURCE:
		format = __resizer_get_format(res, RESZ_PAD_SINK, which);
		fmt->code = format->code;

		crop_rect.left = 0;
		crop_rect.top = 0;
		crop_rect.width = format->width;
		crop_rect.height = format->height;

		ispresizer_calc_ratios(res, &crop_rect, fmt, &ratio);
		break;
	}

	fmt->colorspace = V4L2_COLORSPACE_JPEG;
	fmt->field = V4L2_FIELD_NONE;
	return 0;
}

/**
 * resizer_set_format - Handle set format by pads subdev method
 * @sd    : pointer to v4l2 subdev structure
 * @pad   : pad num
 * @fmt   : pointer to v4l2 format structure
 * @which : wanted subdev format
 * return -EINVAL or zero on success
 */
static int resizer_set_format(struct v4l2_subdev *sd, unsigned int pad,
			      struct v4l2_mbus_framefmt *fmt,
			      enum v4l2_subdev_format which)
{
	struct isp_res_device *res = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format =
		__resizer_get_format(res, pad, which);
	struct resizer_luma_yenh luma = {0, 0, 0, 0};
	int ret = 0;

	if (resizer_try_format(sd, pad, fmt, which) < 0 || format == NULL)
		return -EINVAL;

	*format = *fmt;

	if (which == V4L2_SUBDEV_FORMAT_PROBE)
		return 0;

	switch (pad) {
	case RESZ_PAD_SINK:
		if (res->input == RESIZER_INPUT_VP)
			ispresizer_set_input_offset(res, 0);
		else
			ispresizer_set_input_offset(res, fmt->width * 2);

		/* YUV422 interleaved */
		ispresizer_set_intype(res, RSZ_YUV422);
		ispresizer_set_phase(res, DEFAULT_PHASE, DEFAULT_PHASE);
		/* Disable luminance enhancement algorithm */
		ispresizer_set_luma(res, &luma);
		ispresizer_set_ycpos(res, fmt->code);

		/* reset crop rectangle */
		res->crop.c.left = 0;
		res->crop.c.top = 0;
		res->crop.c.width = fmt->width;
		res->crop.c.height = fmt->height;
		break;

	case RESZ_PAD_SOURCE:
		ispresizer_set_output_offset(res, ALIGN(fmt->width * 2, 32));
		ispresizer_set_output_size(res, fmt->width, fmt->height);

		ispresizer_calc_ratios(res, &res->crop.c, fmt, &res->ratio);
		ispresizer_set_crop_params(res, &res->crop.c, fmt, &res->ratio);
		break;
	}

	return ret;
}

/* subdev core operations */
static const struct v4l2_subdev_core_ops resizer_v4l2_core_ops = {
	.s_power = resizer_s_power,
};

/* subdev video operations */
static const struct v4l2_subdev_video_ops resizer_v4l2_video_ops = {
	.s_stream = resizer_s_stream,
	.cropcap = resizer_cropcap,
	.g_crop = resizer_g_crop,
	.s_crop = resizer_s_crop,
};

/* subdev pad operations */
static const struct v4l2_subdev_pad_ops resizer_v4l2_pad_ops = {
	.enum_mbus_code = resizer_enum_mbus_code,
	.get_fmt = resizer_get_format,
	.set_fmt = resizer_set_format,
};

/* subdev operations */
static const struct v4l2_subdev_ops resizer_v4l2_ops = {
	.core = &resizer_v4l2_core_ops,
	.video = &resizer_v4l2_video_ops,
	.pad = &resizer_v4l2_pad_ops,
};


/* -----------------------------------------------------------------------------
 * Media entity operations
 */

/**
 * resizer_link_setup - Setup resizer connections.
 * @entity : Pointer to media entity structure
 * @local  : Pointer to local pad array
 * @remote : Pointer to remote pad array
 * @flags  : Link flags
 * return -EINVAL or zero on success
 */
static int resizer_link_setup(struct media_entity *entity,
			      const struct media_entity_pad *local,
			      const struct media_entity_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct isp_res_device *res = v4l2_get_subdevdata(sd);

	switch (local->index | (remote->entity->type << 16)) {
	case RESZ_PAD_SINK | (MEDIA_ENTITY_TYPE_NODE << 16):
		/* read from memory */
		if (flags & MEDIA_LINK_FLAG_ACTIVE) {
			if (res->input == RESIZER_INPUT_VP)
				return -EBUSY;
			ispresizer_set_source(res, RSZ_MEM_YUV);
			res->input = RESIZER_INPUT_MEMORY;
		} else {
			if (res->input == RESIZER_INPUT_MEMORY)
				res->input = RESIZER_INPUT_NONE;
		}
		break;

	case RESZ_PAD_SINK | (MEDIA_ENTITY_TYPE_SUBDEV << 16):
		/* read from ccdc or previewer */
		if (flags & MEDIA_LINK_FLAG_ACTIVE) {
			if (res->input == RESIZER_INPUT_MEMORY)
				return -EBUSY;
			ispresizer_set_source(res, RSZ_OTFLY_YUV);
			res->input = RESIZER_INPUT_VP;
		} else {
			if (res->input == RESIZER_INPUT_VP)
				res->input = RESIZER_INPUT_NONE;
		}
		break;

	case RESZ_PAD_SOURCE | (MEDIA_ENTITY_TYPE_NODE << 16):
		/* resizer always write to memory */
		if (flags & MEDIA_LINK_FLAG_ACTIVE)
			res->video = container_of(remote->entity,
					struct isp_video, video.entity);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/* media operations */
static const struct media_entity_operations resizer_media_ops = {
	.link_setup = resizer_link_setup,
	.set_power = v4l2_subdev_set_power,
};

/**
 * ispresizer_init_entities - Initialize resizer subdev and media entity.
 * @res : Pointer to resizer device structure
 * return -ENOMEM or zero on success
 */
static int ispresizer_init_entities(struct isp_res_device *res)
{
	struct v4l2_subdev *sd = &res->subdev;
	struct media_entity_pad *pads = res->pads;
	struct media_entity *me = &sd->entity;
	int ret;

	res->input = RESIZER_INPUT_NONE;

	v4l2_subdev_init(sd, &resizer_v4l2_ops);
	strlcpy(sd->name, "OMAP3 ISP resizer", sizeof(sd->name));
	sd->grp_id = 1 << 16;	/* group ID for isp subdevs */
	v4l2_set_subdevdata(sd, res);

	pads[RESZ_PAD_SINK].type = MEDIA_PAD_TYPE_INPUT;
	pads[RESZ_PAD_SOURCE].type = MEDIA_PAD_TYPE_OUTPUT;

	me->ops = &resizer_media_ops;
	ret = media_entity_init(me, RESZ_PADS_NUM, pads, 0);
	if (ret < 0)
		return ret;

	res->video_in.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	res->video_in.ops = &resizer_video_ops;
	res->video_in.isp = to_isp_device(res);
	res->video_in.capture_mem = PAGE_ALIGN(4096 * 4096) * 2 * 3;
	res->video_in.alignment = 32;
	res->video_out.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	res->video_out.ops = &resizer_video_ops;
	res->video_out.isp = to_isp_device(res);
	res->video_out.capture_mem = PAGE_ALIGN(4096 * 4096) * 2 * 3;
	res->video_out.alignment = 32;

	ret = isp_video_init(&res->video_in, "resizer");
	if (ret < 0)
		return ret;

	ret = isp_video_init(&res->video_out, "resizer");
	if (ret < 0)
		return ret;

	/* Connect the video nodes to the resizer subdev. */
	ret = media_entity_create_link(&res->video_in.video.entity, 0,
			&res->subdev.entity, RESZ_PAD_SINK, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link(&res->subdev.entity, RESZ_PAD_SOURCE,
			&res->video_out.video.entity, 0, 0);
	if (ret < 0)
		return ret;

	return 0;
}

void isp_resizer_unregister_entities(struct isp_res_device *res)
{
	media_entity_cleanup(&res->subdev.entity);

	v4l2_device_unregister_subdev(&res->subdev);
	isp_video_unregister(&res->video_in);
	isp_video_unregister(&res->video_out);
}

int isp_resizer_register_entities(struct isp_res_device *res,
				  struct v4l2_device *vdev)
{
	int ret;

	/* Register the subdev and video nodes. */
	ret = v4l2_device_register_subdev(vdev, &res->subdev);
	if (ret < 0)
		goto error;

	ret = isp_video_register(&res->video_in, vdev);
	if (ret < 0)
		goto error;

	ret = isp_video_register(&res->video_out, vdev);
	if (ret < 0)
		goto error;

	return 0;

error:
	isp_resizer_unregister_entities(res);
	return ret;
}

/* -----------------------------------------------------------------------------
 * ISP resizer initialization and cleanup
 */

void isp_resizer_cleanup(struct isp_device *isp)
{
}

/**
 * isp_resizer_init - Resizer initialization.
 * @isp : Pointer to ISP device
 * return -ENOMEM or zero on success
 */
int isp_resizer_init(struct isp_device *isp)
{
	struct isp_res_device *res = &isp->isp_res;
	int ret;

	ret = ispresizer_init_entities(res);
	if (ret < 0)
		goto out;

out:
	if (ret)
		isp_resizer_cleanup(isp);

	return ret;
}

