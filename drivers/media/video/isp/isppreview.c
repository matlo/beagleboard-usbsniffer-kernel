/*
 * isppreview.c
 *
 * Driver Library for Preview module in TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contributors:
 *	Senthilvadivu Guruswamy <svadivu@ti.com>
 *	Pallavi Kulkarni <p-kulkarni@ti.com>
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

#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/mm.h>

#include "isp.h"
#include "ispreg.h"
#include "isppreview.h"

/* Structure for saving/restoring preview module registers */
static struct isp_reg ispprev_reg_list[] = {
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR, 0x0000}, /* See context saving. */
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_HORZ_INFO, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_VERT_INFO, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_RSDR_ADDR, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_RADR_OFFSET, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_DSDR_ADDR, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_DRKF_OFFSET, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_WSDR_ADDR, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_WADD_OFFSET, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_AVE, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_HMED, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_NF, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_WB_DGAIN, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_WBGAIN, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_WBSEL, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_CFA, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_BLKADJOFF, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_RGB_MAT1, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_RGB_MAT2, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_RGB_MAT3, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_RGB_MAT4, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_RGB_MAT5, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_RGB_OFF1, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_RGB_OFF2, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_CSC0, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_CSC1, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_CSC2, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_CSC_OFFSET, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_CNT_BRT, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_CSUP, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_SETUP_YC, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_CDC_THR0, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_CDC_THR1, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_CDC_THR2, 0x0000},
	{OMAP3_ISP_IOMEM_PREV, ISPPRV_CDC_THR3, 0x0000},
	{0, ISP_TOK_TERM, 0x0000}
};


/* Default values in Office Flourescent Light for RGBtoRGB Blending */
static struct ispprev_rgbtorgb flr_rgb2rgb = {
	{	/* RGB-RGB Matrix */
		{0x01E2, 0x0F30, 0x0FEE},
		{0x0F9B, 0x01AC, 0x0FB9},
		{0x0FE0, 0x0EC0, 0x0260}
	},	/* RGB Offset */
	{0x0000, 0x0000, 0x0000}
};

/* Default values in Office Flourescent Light for RGB to YUV Conversion*/
static struct ispprev_csc flr_prev_csc = {
	{	/* CSC Coef Matrix */
		{66, 129, 25},
		{-38, -75, 112},
		{112, -94 , -18}
	},	/* CSC Offset */
	{0x0, 0x0, 0x0}
};


/* Default values in Office Flourescent Light for CFA Gradient*/
#define FLR_CFA_GRADTHRS_HORZ	0x28
#define FLR_CFA_GRADTHRS_VERT	0x28

/* Default values in Office Flourescent Light for Chroma Suppression*/
#define FLR_CSUP_GAIN		0x0D
#define FLR_CSUP_THRES		0xEB

/* Default values in Office Flourescent Light for Noise Filter*/
#define FLR_NF_STRGTH		0x03

/* Default values in Office Flourescent Light for White Balance*/
#define FLR_WBAL_DGAIN		0x100
#define FLR_WBAL_COEF0		0x20
#define FLR_WBAL_COEF1		0x29
#define FLR_WBAL_COEF2		0x2d
#define FLR_WBAL_COEF3		0x20

#define FLR_WBAL_COEF0_ES1	0x20
#define FLR_WBAL_COEF1_ES1	0x23
#define FLR_WBAL_COEF2_ES1	0x39
#define FLR_WBAL_COEF3_ES1	0x20

/* Default values in Office Flourescent Light for Black Adjustment*/
#define FLR_BLKADJ_BLUE		0x0
#define FLR_BLKADJ_GREEN	0x0
#define FLR_BLKADJ_RED		0x0

#define DEF_DETECT_CORRECT_VAL	0xe

#define PREV_MIN_WIDTH		64
#define PREV_MIN_HEIGHT		64
#define PREV_MAX_HEIGHT		0x3fff

/*
 * Coeficient Tables for the submodules in Preview.
 * Array is initialised with the values from.the tables text file.
 */

/*
 * CFA Filter Coefficient Table
 *
 */
static u32 cfa_coef_table[] = {
#include "cfa_coef_table.h"
};

/*
 * Gamma Correction Table - Red
 */
static u32 redgamma_table[] = {
#include "redgamma_table.h"
};

/*
 * Gamma Correction Table - Green
 */
static u32 greengamma_table[] = {
#include "greengamma_table.h"
};

/*
 * Gamma Correction Table - Blue
 */
static u32 bluegamma_table[] = {
#include "bluegamma_table.h"
};

/*
 * Noise Filter Threshold table
 */
static u32 noise_filter_table[] = {
#include "noise_filter_table.h"
};

/*
 * Luminance Enhancement Table
 */
static u32 luma_enhance_table[] = {
#include "luma_enhance_table.h"
};

/**
 * isppreview_enable_invalaw - Enable/Disable Inverse A-Law module in Preview.
 * @enable: 1 - Reverse the A-Law done in CCDC.
 **/
static void
isppreview_enable_invalaw(struct isp_prev_device *prev, u8 enable)
{
	struct isp_device *isp = to_isp_device(prev);
	u32 pcr_val = 0;

	pcr_val = isp_reg_readl(isp,
				OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR);

	if (enable) {
		isp_reg_writel(isp,
			       pcr_val | ISPPRV_PCR_WIDTH | ISPPRV_PCR_INVALAW,
			       OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR);
	} else {
		isp_reg_writel(isp, pcr_val &
			       ~(ISPPRV_PCR_WIDTH | ISPPRV_PCR_INVALAW),
			       OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR);
	}
}

/**
 * isppreview_enable_drkframe_capture - Enable/Disable of the darkframe capture.
 * @prev -
 * @enable: 1 - Enable, 0 - Disable
 *
 * NOTE: PRV_WSDR_ADDR and PRV_WADD_OFFSET must be set also
 * The proccess is applied for each captured frame.
 **/
static void
isppreview_enable_drkframe_capture(struct isp_prev_device *prev, u8 enable)
{
	struct isp_device *isp = to_isp_device(prev);

	if (enable)
		isp_reg_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			   ISPPRV_PCR_DRKFCAP);
	else
		isp_reg_and(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			    ~ISPPRV_PCR_DRKFCAP);
}

/**
 * isppreview_enable_drkframe - Enable/Disable of the darkframe subtract.
 * @enable: 1 - Acquires memory bandwidth since the pixels in each frame is
 *          subtracted with the pixels in the current frame.
 *
 * The proccess is applied for each captured frame.
 **/
static void
isppreview_enable_drkframe(struct isp_prev_device *prev, u8 enable)
{
	struct isp_device *isp = to_isp_device(prev);

	if (enable)
		isp_reg_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			   ISPPRV_PCR_DRKFEN);
	else
		isp_reg_and(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			    ~ISPPRV_PCR_DRKFEN);
}

/**
 * isppreview_config_drkf_shadcomp - Configures shift value in shading comp.
 * @scomp_shtval: 3bit value of shift used in shading compensation.
 **/
static void
isppreview_config_drkf_shadcomp(struct isp_prev_device *prev,
				const void *scomp_shtval)
{
	struct isp_device *isp = to_isp_device(prev);
	const u32 *shtval = scomp_shtval;
	u32 pcr_val = isp_reg_readl(isp,
				    OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR);

	pcr_val &= ISPPRV_PCR_SCOMP_SFT_MASK;
	isp_reg_writel(isp,
		       pcr_val | (*shtval << ISPPRV_PCR_SCOMP_SFT_SHIFT),
		       OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR);
}

/**
 * isppreview_enable_hmed - Enables/Disables of the Horizontal Median Filter.
 * @enable: 1 - Enables Horizontal Median Filter.
 **/
static void
isppreview_enable_hmed(struct isp_prev_device *prev, u8 enable)
{
	struct isp_device *isp = to_isp_device(prev);

	if (enable)
		isp_reg_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			   ISPPRV_PCR_HMEDEN);
	else
		isp_reg_and(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			    ~ISPPRV_PCR_HMEDEN);
}

/**
 * isppreview_config_hmed - Configures the Horizontal Median Filter.
 * @prev_hmed: Structure containing the odd and even distance between the
 *             pixels in the image along with the filter threshold.
 **/
static void
isppreview_config_hmed(struct isp_prev_device *prev, const void *prev_hmed)
{
	struct isp_device *isp = to_isp_device(prev);
	const struct ispprev_hmed *hmed = prev_hmed;

	u32 odddist = 0;
	u32 evendist = 0;

	if (hmed->odddist == 1)
		odddist = ~ISPPRV_HMED_ODDDIST;
	else
		odddist = ISPPRV_HMED_ODDDIST;

	if (hmed->evendist == 1)
		evendist = ~ISPPRV_HMED_EVENDIST;
	else
		evendist = ISPPRV_HMED_EVENDIST;

	isp_reg_writel(isp, odddist | evendist | (hmed->thres <<
					     ISPPRV_HMED_THRESHOLD_SHIFT),
		       OMAP3_ISP_IOMEM_PREV, ISPPRV_HMED);

}

/**
 * isppreview_config_noisefilter - Configures the Noise Filter.
 * @prev_nf: Structure containing the noisefilter table, strength to be used
 *           for the noise filter and the defect correction enable flag.
 **/
static void
isppreview_config_noisefilter(struct isp_prev_device *prev, const void *prev_nf)
{
	struct isp_device *isp = to_isp_device(prev);
	const struct ispprev_nf *nf = prev_nf;
	int i = 0;

	isp_reg_writel(isp, nf->spread, OMAP3_ISP_IOMEM_PREV,
		       ISPPRV_NF);
	isp_reg_writel(isp, ISPPRV_NF_TABLE_ADDR,
		       OMAP3_ISP_IOMEM_PREV, ISPPRV_SET_TBL_ADDR);
	for (i = 0; i < ISPPRV_NF_TBL_SIZE; i++) {
		isp_reg_writel(isp, nf->table[i],
			       OMAP3_ISP_IOMEM_PREV, ISPPRV_SET_TBL_DATA);
	}
}

/**
 * isppreview_config_dcor - Configures the defect correction
 * @prev_nf: Structure containing the defect correction structure
 **/
static void
isppreview_config_dcor(struct isp_prev_device *prev, const void *prev_dcor)
{
	struct isp_device *isp = to_isp_device(prev);
	const struct ispprev_dcor *dcor = prev_dcor;

	if (dcor->couplet_mode_en) {
		isp_reg_writel(isp, dcor->detect_correct[0],
			       OMAP3_ISP_IOMEM_PREV, ISPPRV_CDC_THR0);
		isp_reg_writel(isp, dcor->detect_correct[1],
			       OMAP3_ISP_IOMEM_PREV, ISPPRV_CDC_THR1);
		isp_reg_writel(isp, dcor->detect_correct[2],
			       OMAP3_ISP_IOMEM_PREV, ISPPRV_CDC_THR2);
		isp_reg_writel(isp, dcor->detect_correct[3],
			       OMAP3_ISP_IOMEM_PREV, ISPPRV_CDC_THR3);
		isp_reg_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			   ISPPRV_PCR_DCCOUP);
	} else {
		isp_reg_and(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			    ~ISPPRV_PCR_DCCOUP);
	}
}

/**
 * isppreview_config_cfa - Configures the CFA Interpolation parameters.
 * @prev_cfa: Structure containing the CFA interpolation table, CFA format
 *            in the image, vertical and horizontal gradient threshold.
 **/
static void
isppreview_config_cfa(struct isp_prev_device *prev, const void *prev_cfa)
{
	struct isp_device *isp = to_isp_device(prev);
	const struct ispprev_cfa *cfa = prev_cfa;
	int i = 0;

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
		       ~ISPPRV_PCR_CFAFMT_MASK,
		       (cfa->format << ISPPRV_PCR_CFAFMT_SHIFT));

	isp_reg_writel(isp,
		(cfa->gradthrs_vert << ISPPRV_CFA_GRADTH_VER_SHIFT) |
		(cfa->gradthrs_horz << ISPPRV_CFA_GRADTH_HOR_SHIFT),
		OMAP3_ISP_IOMEM_PREV, ISPPRV_CFA);

	isp_reg_writel(isp, ISPPRV_CFA_TABLE_ADDR,
		       OMAP3_ISP_IOMEM_PREV, ISPPRV_SET_TBL_ADDR);

	for (i = 0; i < ISPPRV_CFA_TBL_SIZE; i++) {
		isp_reg_writel(isp, cfa->table[i],
			       OMAP3_ISP_IOMEM_PREV, ISPPRV_SET_TBL_DATA);
	}
}

/**
 * isppreview_config_gammacorrn - Configures the Gamma Correction table values
 * @gtable: Structure containing the table for red, blue, green gamma table.
 **/
static void
isppreview_config_gammacorrn(struct isp_prev_device *prev, const void *gtable)
{
	struct isp_device *isp = to_isp_device(prev);
	const struct ispprev_gtables *gt = gtable;
	int i = 0;

	if (gt->red) {
		isp_reg_writel(isp, ISPPRV_REDGAMMA_TABLE_ADDR,
			       OMAP3_ISP_IOMEM_PREV, ISPPRV_SET_TBL_ADDR);
		for (i = 0; i < ISPPRV_GAMMA_TBL_SIZE; i++) {
			isp_reg_writel(isp, gt->red[i],
				       OMAP3_ISP_IOMEM_PREV,
				       ISPPRV_SET_TBL_DATA);
		}
	}
	if (gt->green) {
		isp_reg_writel(isp, ISPPRV_GREENGAMMA_TABLE_ADDR,
			       OMAP3_ISP_IOMEM_PREV, ISPPRV_SET_TBL_ADDR);
		for (i = 0; i < ISPPRV_GAMMA_TBL_SIZE; i++) {
			isp_reg_writel(isp, gt->green[i],
				       OMAP3_ISP_IOMEM_PREV,
				       ISPPRV_SET_TBL_DATA);
		}
	}
	if (gt->blue) {
		isp_reg_writel(isp, ISPPRV_BLUEGAMMA_TABLE_ADDR,
			       OMAP3_ISP_IOMEM_PREV, ISPPRV_SET_TBL_ADDR);
		for (i = 0; i < ISPPRV_GAMMA_TBL_SIZE; i++) {
			isp_reg_writel(isp, gt->blue[i],
				       OMAP3_ISP_IOMEM_PREV,
				       ISPPRV_SET_TBL_DATA);
		}
	}
}

/**
 * isppreview_config_luma_enhancement - Sets the Luminance Enhancement table.
 * @ytable: Structure containing the table for Luminance Enhancement table.
 **/
static void
isppreview_config_luma_enhancement(struct isp_prev_device *prev,
				   const void *ytable)
{
	struct isp_device *isp = to_isp_device(prev);
	const struct ispprev_luma *yt = ytable;
	int i = 0;

	isp_reg_writel(isp, ISPPRV_YENH_TABLE_ADDR,
		       OMAP3_ISP_IOMEM_PREV, ISPPRV_SET_TBL_ADDR);
	for (i = 0; i < ISPPRV_YENH_TBL_SIZE; i++) {
		isp_reg_writel(isp, yt->table[i],
			       OMAP3_ISP_IOMEM_PREV, ISPPRV_SET_TBL_DATA);
	}
}

/**
 * isppreview_config_chroma_suppression - Configures the Chroma Suppression.
 * @csup: Structure containing the threshold value for suppression
 *        and the hypass filter enable flag.
 **/
static void
isppreview_config_chroma_suppression(struct isp_prev_device *prev,
				     const void *csup)
{
	struct isp_device *isp = to_isp_device(prev);
	const struct ispprev_csup *cs = csup;

	isp_reg_writel(isp,
		       cs->gain | (cs->thres << ISPPRV_CSUP_THRES_SHIFT) |
		       (cs->hypf_en << ISPPRV_CSUP_HPYF_SHIFT),
		       OMAP3_ISP_IOMEM_PREV, ISPPRV_CSUP);
}

/**
 * isppreview_enable_noisefilter - Enables/Disables the Noise Filter.
 * @enable: 1 - Enables the Noise Filter.
 **/
static void
isppreview_enable_noisefilter(struct isp_prev_device *prev, u8 enable)
{
	struct isp_device *isp = to_isp_device(prev);

	if (enable)
		isp_reg_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			   ISPPRV_PCR_NFEN);
	else
		isp_reg_and(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			    ~ISPPRV_PCR_NFEN);
}

/**
 * isppreview_enable_dcor - Enables/Disables the defect correction.
 * @enable: 1 - Enables the defect correction.
 **/
static void
isppreview_enable_dcor(struct isp_prev_device *prev, u8 enable)
{
	struct isp_device *isp = to_isp_device(prev);

	if (enable)
		isp_reg_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			   ISPPRV_PCR_DCOREN);
	else {
		isp_reg_and(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			    ~ISPPRV_PCR_DCOREN);
	}
}

/**
 * isppreview_enable_cfa - Enable/Disable the CFA Interpolation.
 * @enable: 1 - Enables the CFA.
 **/
static void
isppreview_enable_cfa(struct isp_prev_device *prev, u8 enable)
{
	struct isp_device *isp = to_isp_device(prev);

	if (enable)
		isp_reg_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			   ISPPRV_PCR_CFAEN);
	else {
		isp_reg_and(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			    ~ISPPRV_PCR_CFAEN);
	}
}

/**
 * isppreview_enable_gammabypass - Enables/Disables the GammaByPass
 * @enable: 1 - Bypasses Gamma - 10bit input is cropped to 8MSB.
 *          0 - Goes through Gamma Correction. input and output is 10bit.
 **/
static void
isppreview_enable_gammabypass(struct isp_prev_device *prev, u8 enable)
{
	struct isp_device *isp = to_isp_device(prev);

	if (enable) {
		isp_reg_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			   ISPPRV_PCR_GAMMA_BYPASS);
	} else {
		isp_reg_and(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			    ~ISPPRV_PCR_GAMMA_BYPASS);
	}
}

/**
 * isppreview_enable_luma_enhancement - Enables/Disables Luminance Enhancement
 * @enable: 1 - Enable the Luminance Enhancement.
 **/
static void
isppreview_enable_luma_enhancement(struct isp_prev_device *prev, u8 enable)
{
	struct isp_device *isp = to_isp_device(prev);

	if (enable) {
		isp_reg_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			   ISPPRV_PCR_YNENHEN);
	} else {
		isp_reg_and(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			    ~ISPPRV_PCR_YNENHEN);
	}
}

/**
 * isppreview_enable_chroma_suppression - Enables/Disables Chrominance Suppr.
 * @enable: 1 - Enable the Chrominance Suppression.
 **/
static void
isppreview_enable_chroma_suppression(struct isp_prev_device *prev, u8 enable)
{
	struct isp_device *isp = to_isp_device(prev);

	if (enable)
		isp_reg_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			   ISPPRV_PCR_SUPEN);
	else {
		isp_reg_and(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
			    ~ISPPRV_PCR_SUPEN);
	}
}

/**
 * isppreview_config_whitebalance - Configures the White Balance parameters.
 * @prev_wbal: Structure containing the digital gain and white balance
 *             coefficient.
 *
 * Coefficient matrix always with default values.
 **/
void
isppreview_config_whitebalance(struct isp_prev_device *prev,
			       const void *prev_wbal)
{
	struct isp_device *isp = to_isp_device(prev);
	const struct ispprev_wbal *wbal = prev_wbal;
	u32 val;

	isp_reg_writel(isp, wbal->dgain, OMAP3_ISP_IOMEM_PREV,
		       ISPPRV_WB_DGAIN);

	val = wbal->coef0 << ISPPRV_WBGAIN_COEF0_SHIFT;
	val |= wbal->coef1 << ISPPRV_WBGAIN_COEF1_SHIFT;
	val |= wbal->coef2 << ISPPRV_WBGAIN_COEF2_SHIFT;
	val |= wbal->coef3 << ISPPRV_WBGAIN_COEF3_SHIFT;
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_PREV,
		       ISPPRV_WBGAIN);

	isp_reg_writel(isp,
		       ISPPRV_WBSEL_COEF0 << ISPPRV_WBSEL_N0_0_SHIFT |
		       ISPPRV_WBSEL_COEF1 << ISPPRV_WBSEL_N0_1_SHIFT |
		       ISPPRV_WBSEL_COEF0 << ISPPRV_WBSEL_N0_2_SHIFT |
		       ISPPRV_WBSEL_COEF1 << ISPPRV_WBSEL_N0_3_SHIFT |
		       ISPPRV_WBSEL_COEF2 << ISPPRV_WBSEL_N1_0_SHIFT |
		       ISPPRV_WBSEL_COEF3 << ISPPRV_WBSEL_N1_1_SHIFT |
		       ISPPRV_WBSEL_COEF2 << ISPPRV_WBSEL_N1_2_SHIFT |
		       ISPPRV_WBSEL_COEF3 << ISPPRV_WBSEL_N1_3_SHIFT |
		       ISPPRV_WBSEL_COEF0 << ISPPRV_WBSEL_N2_0_SHIFT |
		       ISPPRV_WBSEL_COEF1 << ISPPRV_WBSEL_N2_1_SHIFT |
		       ISPPRV_WBSEL_COEF0 << ISPPRV_WBSEL_N2_2_SHIFT |
		       ISPPRV_WBSEL_COEF1 << ISPPRV_WBSEL_N2_3_SHIFT |
		       ISPPRV_WBSEL_COEF2 << ISPPRV_WBSEL_N3_0_SHIFT |
		       ISPPRV_WBSEL_COEF3 << ISPPRV_WBSEL_N3_1_SHIFT |
		       ISPPRV_WBSEL_COEF2 << ISPPRV_WBSEL_N3_2_SHIFT |
		       ISPPRV_WBSEL_COEF3 << ISPPRV_WBSEL_N3_3_SHIFT,
		       OMAP3_ISP_IOMEM_PREV, ISPPRV_WBSEL);
}

/**
 * isppreview_config_blkadj - Configures the Black Adjustment parameters.
 * @prev_blkadj: Structure containing the black adjustment towards red, green,
 *               blue.
 **/
static void
isppreview_config_blkadj(struct isp_prev_device *prev, const void *prev_blkadj)
{
	struct isp_device *isp = to_isp_device(prev);
	const struct ispprev_blkadj *blkadj = prev_blkadj;

	isp_reg_writel(isp, blkadj->blue |
		       (blkadj->green << ISPPRV_BLKADJOFF_G_SHIFT) |
		       (blkadj->red << ISPPRV_BLKADJOFF_R_SHIFT),
		       OMAP3_ISP_IOMEM_PREV, ISPPRV_BLKADJOFF);
}

/**
 * isppreview_config_rgb_blending - Configures the RGB-RGB Blending matrix.
 * @rgb2rgb: Structure containing the rgb to rgb blending matrix and the rgb
 *           offset.
 **/
static void
isppreview_config_rgb_blending(struct isp_prev_device *prev,
			       const void *rgb2rgb)
{
	struct isp_device *isp = to_isp_device(prev);
	const struct ispprev_rgbtorgb *rgbrgb = rgb2rgb;
	u32 val = 0;

	val = (rgbrgb->matrix[0][0] & 0xfff) << ISPPRV_RGB_MAT1_MTX_RR_SHIFT;
	val |= (rgbrgb->matrix[0][1] & 0xfff) << ISPPRV_RGB_MAT1_MTX_GR_SHIFT;
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_PREV,
		       ISPPRV_RGB_MAT1);

	val = (rgbrgb->matrix[0][2] & 0xfff) << ISPPRV_RGB_MAT2_MTX_BR_SHIFT;
	val |= (rgbrgb->matrix[1][0] & 0xfff) << ISPPRV_RGB_MAT2_MTX_RG_SHIFT;
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_PREV,
		       ISPPRV_RGB_MAT2);

	val = (rgbrgb->matrix[1][1] & 0xfff) << ISPPRV_RGB_MAT3_MTX_GG_SHIFT;
	val |= (rgbrgb->matrix[1][2] & 0xfff) << ISPPRV_RGB_MAT3_MTX_BG_SHIFT;
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_PREV,
		       ISPPRV_RGB_MAT3);

	val = (rgbrgb->matrix[2][0] & 0xfff) << ISPPRV_RGB_MAT4_MTX_RB_SHIFT;
	val |= (rgbrgb->matrix[2][1] & 0xfff) << ISPPRV_RGB_MAT4_MTX_GB_SHIFT;
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_PREV,
		       ISPPRV_RGB_MAT4);

	val = (rgbrgb->matrix[2][2] & 0xfff) << ISPPRV_RGB_MAT5_MTX_BB_SHIFT;
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_PREV,
		       ISPPRV_RGB_MAT5);

	val = (rgbrgb->offset[0] & 0x3ff) << ISPPRV_RGB_OFF1_MTX_OFFR_SHIFT;
	val |= (rgbrgb->offset[1] & 0x3ff) << ISPPRV_RGB_OFF1_MTX_OFFG_SHIFT;
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_PREV,
		       ISPPRV_RGB_OFF1);

	val = (rgbrgb->offset[2] & 0x3ff) << ISPPRV_RGB_OFF2_MTX_OFFB_SHIFT;
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_PREV,
		       ISPPRV_RGB_OFF2);
}

/**
 * Configures the RGB-YCbYCr conversion matrix
 * @prev_csc: Structure containing the RGB to YCbYCr matrix and the
 *            YCbCr offset.
 **/
static void
isppreview_config_rgb_to_ycbcr(struct isp_prev_device *prev,
			       const void *prev_csc)
{
	struct isp_device *isp = to_isp_device(prev);
	const struct ispprev_csc *csc = prev_csc;
	u32 val = 0;

	val = (csc->matrix[0][0] & 0x3ff) << ISPPRV_CSC0_RY_SHIFT;
	val |= (csc->matrix[0][1] & 0x3ff) << ISPPRV_CSC0_GY_SHIFT;
	val |= (csc->matrix[0][2] & 0x3ff) << ISPPRV_CSC0_BY_SHIFT;
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_PREV, ISPPRV_CSC0);

	val = (csc->matrix[1][0] & 0x3ff) << ISPPRV_CSC1_RCB_SHIFT;
	val |= (csc->matrix[1][1] & 0x3ff) << ISPPRV_CSC1_GCB_SHIFT;
	val |= (csc->matrix[1][2] & 0x3ff) << ISPPRV_CSC1_BCB_SHIFT;
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_PREV, ISPPRV_CSC1);

	val = (csc->matrix[2][0] & 0x3ff) << ISPPRV_CSC2_RCR_SHIFT;
	val |= (csc->matrix[2][1] & 0x3ff) << ISPPRV_CSC2_GCR_SHIFT;
	val |= (csc->matrix[2][2] & 0x3ff) << ISPPRV_CSC2_BCR_SHIFT;
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_PREV, ISPPRV_CSC2);

	val = (csc->offset[0] & 0xff) << ISPPRV_CSC_OFFSET_Y_SHIFT;
	val |= (csc->offset[1] & 0xff) << ISPPRV_CSC_OFFSET_CB_SHIFT;
	val |= (csc->offset[2] & 0xff) << ISPPRV_CSC_OFFSET_CR_SHIFT;
	isp_reg_writel(isp, val, OMAP3_ISP_IOMEM_PREV,
		       ISPPRV_CSC_OFFSET);
}

/**
 * isppreview_query_contrast - Query the contrast.
 * @contrast: Pointer to hold the current programmed contrast value.
 **/
static void
isppreview_query_contrast(struct isp_prev_device *prev, u8 *contrast)
{
	*contrast = prev->params.contrast / ISPPRV_CONTRAST_UNITS;
}

/**
 * isppreview_update_contrast - Updates the contrast.
 * @contrast: Pointer to hold the current programmed contrast value.
 *
 * Value should be programmed before enabling the module.
 **/
static int
isppreview_update_contrast(struct isp_prev_device *prev, u8 contrast)
{
	struct prev_params *params = &prev->params;

	if (contrast > ISPPRV_CONTRAST_HIGH)
		return -EINVAL;
	if (params->contrast != (contrast * ISPPRV_CONTRAST_UNITS)) {
		params->contrast = contrast * ISPPRV_CONTRAST_UNITS;
		prev->update |= PREV_CONTRAST;
	}
	return 0;
}

/**
 * isppreview_config_contrast - Configures the Contrast.
 * @contrast: 8 bit value in U8Q4 format.
 *
 * Value should be programmed before enabling the module.
 **/
static void
isppreview_config_contrast(struct isp_prev_device *prev, u8 contrast)
{
	struct isp_device *isp = to_isp_device(prev);
	u32 brt_cnt_val = 0;

	brt_cnt_val = isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV,
				    ISPPRV_CNT_BRT);
	brt_cnt_val &= ~(0xff << ISPPRV_CNT_BRT_CNT_SHIFT);
	isp_reg_writel(isp,
		       brt_cnt_val | contrast << ISPPRV_CNT_BRT_CNT_SHIFT,
		       OMAP3_ISP_IOMEM_PREV, ISPPRV_CNT_BRT);
}

/**
 * isppreview_update_brightness - Updates the brightness in preview module.
 * @brightness: Pointer to hold the current programmed brightness value.
 *
 **/
static int
isppreview_update_brightness(struct isp_prev_device *prev, u8 brightness)
{
	struct prev_params *params = &prev->params;

	if (brightness > ISPPRV_BRIGHT_HIGH)
		return -EINVAL;
	if (params->brightness != (brightness * ISPPRV_BRIGHT_UNITS)) {
		params->brightness = brightness * ISPPRV_BRIGHT_UNITS;
		prev->update |= PREV_BRIGHTNESS;
	}
	return 0;
}

/**
 * isppreview_config_brightness - Configures the brightness.
 * @contrast: 8bitvalue in U8Q0 format.
 **/
static void
isppreview_config_brightness(struct isp_prev_device *prev, u8 brightness)
{
	struct isp_device *isp = to_isp_device(prev);
	u32 brt_cnt_val = 0;

	dev_dbg(isp->dev, "\tPREV: Configuring brightness in ISP: %d\n",
		brightness);
	brt_cnt_val = isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV,
				    ISPPRV_CNT_BRT);
	brt_cnt_val &= ~(0xff << ISPPRV_CNT_BRT_BRT_SHIFT);
	isp_reg_writel(isp,
		       brt_cnt_val | brightness << ISPPRV_CNT_BRT_BRT_SHIFT,
		       OMAP3_ISP_IOMEM_PREV, ISPPRV_CNT_BRT);
}

/**
 * isppreview_query_brightness - Query the brightness.
 * @brightness: Pointer to hold the current programmed brightness value.
 **/
static void
isppreview_query_brightness(struct isp_prev_device *prev, u8 *brightness)
{
	*brightness = prev->params.brightness / ISPPRV_BRIGHT_UNITS;
}

/**
 * isppreview_config_yc_range - Configures the max and min Y and C values.
 * @yclimit: Structure containing the range of Y and C values.
 **/
static void
isppreview_config_yc_range(struct isp_prev_device *prev, const void *yclimit)
{
	struct isp_device *isp = to_isp_device(prev);
	const struct ispprev_yclimit *yc = yclimit;

	isp_reg_writel(isp,
		       yc->maxC << ISPPRV_SETUP_YC_MAXC_SHIFT |
		       yc->maxY << ISPPRV_SETUP_YC_MAXY_SHIFT |
		       yc->minC << ISPPRV_SETUP_YC_MINC_SHIFT |
		       yc->minY << ISPPRV_SETUP_YC_MINY_SHIFT,
		       OMAP3_ISP_IOMEM_PREV, ISPPRV_SETUP_YC);
}

/* preview parameters update structure */
struct preview_update {
	int cfg_bit;
	int feature_bit;
	void (*config)(struct isp_prev_device *, const void *);
	void (*enable)(struct isp_prev_device *, u8);
};

static struct preview_update update_attrs[] = {
	{ISP_PREV_LUMAENH, PREV_LUMA_ENHANCE,
		isppreview_config_luma_enhancement,
		isppreview_enable_luma_enhancement},
	{ISP_PREV_INVALAW, PREV_INVERSE_ALAW,
		NULL,
		isppreview_enable_invalaw},
	{ISP_PREV_HRZ_MED, PREV_HORZ_MEDIAN_FILTER,
		isppreview_config_hmed,
		isppreview_enable_hmed},
	{ISP_PREV_CFA, PREV_CFA,
		isppreview_config_cfa,
		isppreview_enable_cfa},
	{ISP_PREV_CHROMA_SUPP, PREV_CHROMA_SUPPRESS,
		isppreview_config_chroma_suppression,
		isppreview_enable_chroma_suppression},
	{ISP_PREV_WB, PREV_WB,
		isppreview_config_whitebalance,
		NULL},
	{ISP_PREV_BLKADJ, PREV_BLKADJ,
		isppreview_config_blkadj,
		NULL},
	{ISP_PREV_RGB2RGB, PREV_RGB2RGB,
		isppreview_config_rgb_blending,
		NULL},
	{ISP_PREV_COLOR_CONV, PREV_COLOR_CONV,
		isppreview_config_rgb_to_ycbcr,
		NULL},
	{ISP_PREV_YC_LIMIT, PREV_YCLIMITS,
		isppreview_config_yc_range,
		NULL},
	{ISP_PREV_DEFECT_COR, PREV_DEFECT_COR,
		isppreview_config_dcor,
		isppreview_enable_dcor},
	{ISP_PREV_GAMMABYPASS, PREV_GAMMA_BYPASS,
		NULL,
		isppreview_enable_gammabypass},
	{ISP_PREV_DRK_FRM_CAPTURE, PREV_DARK_FRAME_CAPTURE,
		NULL,
		isppreview_enable_drkframe_capture},
	{ISP_PREV_DRK_FRM_SUBTRACT, PREV_DARK_FRAME_SUBTRACT,
		NULL,
		isppreview_enable_drkframe},
	{ISP_PREV_LENS_SHADING, PREV_LENS_SHADING,
		isppreview_config_drkf_shadcomp,
		isppreview_enable_drkframe},
	{ISP_PREV_NF, PREV_NOISE_FILTER,
		isppreview_config_noisefilter,
		isppreview_enable_noisefilter},
	{ISP_PREV_GAMMA, PREV_GAMMA,
		isppreview_config_gammacorrn,
		NULL},
};

/**
 * __isppreview_get_ptrs - helper function which return pointers to members
 *                         of params and config structures.
 * @params - pointer to preview_params structure.
 * @param - return pointer to appropriate structure field.
 * @configs - pointer to update config structure.
 * @config - return pointer to appropriate structure field.
 * @bit - for which feature to return pointers.
 * Return size of coresponding prev_params member
 **/
static u32
__isppreview_get_ptrs(struct prev_params *params, void **param,
		      struct ispprv_update_config *configs,
		      void __user **config,
		      u32 bit)
{
#define CHKARG(cfgs, cfg, field)				\
	if (cfgs && cfg) {					\
		*(cfg) = (cfgs)->field;				\
	}

	switch (bit) {
	case PREV_HORZ_MEDIAN_FILTER:
		*param = &params->hmed;
		CHKARG(configs, config, hmed)
		return sizeof(params->hmed);
	case PREV_NOISE_FILTER:
		*param = &params->nf;
		CHKARG(configs, config, nf)
		return sizeof(params->nf);
		break;
	case PREV_CFA:
		*param = &params->cfa;
		CHKARG(configs, config, cfa)
		return sizeof(params->cfa);
	case PREV_LUMA_ENHANCE:
		*param = &params->luma;
		CHKARG(configs, config, luma)
		return sizeof(params->luma);
	case PREV_CHROMA_SUPPRESS:
		*param = &params->csup;
		CHKARG(configs, config, csup)
		return sizeof(params->csup);
	case PREV_DEFECT_COR:
		*param = &params->dcor;
		CHKARG(configs, config, dcor)
		return sizeof(params->dcor);
	case PREV_BLKADJ:
		*param = &params->blk_adj;
		CHKARG(configs, config, blkadj)
		return sizeof(params->blk_adj);
	case PREV_YCLIMITS:
		*param = &params->yclimit;
		CHKARG(configs, config, yclimit)
		return sizeof(params->yclimit);
	case PREV_RGB2RGB:
		*param = &params->rgb2rgb;
		CHKARG(configs, config, rgb2rgb)
		return sizeof(params->rgb2rgb);
	case PREV_COLOR_CONV:
		*param = &params->rgb2ycbcr;
		CHKARG(configs, config, csc)
		return sizeof(params->rgb2ycbcr);
	case PREV_WB:
		*param = &params->wbal;
		CHKARG(configs, config, wbal)
		return sizeof(params->wbal);
	case PREV_GAMMA:
		*param = &params->gamma;
		CHKARG(configs, config, gamma)
		return sizeof(params->gamma);
	default:
		*param = NULL;
		*config = NULL;
		break;
	}
	return 0;
}

/*
 * isppreview_config - Copy and update local structure with userspace preview
 *                     configuration.
 * @prev: ISP preview engine
 * @cfg: Configuration
 *
 * Return zero if success or -EFAULT if the configuration can't be copied from
 * userspace.
 */
static int isppreview_config(struct isp_prev_device *prev,
			     struct ispprv_update_config *cfg)
{
	struct prev_params *params;
	struct preview_update *attr;
	int i, bit, rval = 0;

	params = &prev->params;

	if (prev->state != ISP_PIPELINE_STREAM_STOPPED) {
		unsigned long flags;

		spin_lock_irqsave(&prev->lock, flags);
		prev->shadow_update = 1;
		spin_unlock_irqrestore(&prev->lock, flags);
	}

	for (i = 0; i < ARRAY_SIZE(update_attrs); i++) {
		attr = &update_attrs[i];
		bit = 0;

		if (!(cfg->update & attr->cfg_bit))
			continue;

		bit = cfg->flag & attr->cfg_bit;
		if (bit) {
			void *to = NULL, __user *from = NULL;
			unsigned long sz = 0;

			sz = __isppreview_get_ptrs(params, &to, cfg, &from,
						   bit);
			if (to && from && sz) {
				if (copy_from_user(to, from, sz)) {
					rval = -EFAULT;
					break;
				}
			}
			params->features |= attr->feature_bit;
		} else {
			params->features &= ~attr->feature_bit;
		}

		prev->update |= attr->feature_bit;
	}

	prev->shadow_update = 0;
	return rval;
}

/**
 * isppreview_setup_hw - Setup preview registers and/or internal memory
 * @prev: pointer to preview private structure
 * Note: can be called from interrupt context
 * Return none
 **/
static void isppreview_setup_hw(struct isp_prev_device *prev)
{
	struct prev_params *params = &prev->params;
	struct preview_update *attr;
	int i, bit;
	void *param_ptr;

	for (i = 0; i < ARRAY_SIZE(update_attrs); i++) {
		attr = &update_attrs[i];

		if (!(prev->update & attr->feature_bit))
			continue;
		bit = params->features & attr->feature_bit;
		if (bit) {
			if (attr->config) {
				__isppreview_get_ptrs(params, &param_ptr, NULL,
						      NULL, bit);
				attr->config(prev, param_ptr);
			}
			if (attr->enable)
				attr->enable(prev, 1);
		} else
			if (attr->enable)
				attr->enable(prev, 0);

		prev->update &= ~attr->feature_bit;
	}
}

/**
 * isppreview_config_ycpos - Configure byte layout of YUV image.
 * @mode: Indicates the required byte layout.
 **/
static void
isppreview_config_ycpos(struct isp_prev_device *prev,
			enum v4l2_mbus_pixelcode pixelcode)
{
	struct isp_device *isp = to_isp_device(prev);
	enum preview_ycpos_mode mode;
	u32 pcr;

	switch (pixelcode) {
	case V4L2_MBUS_FMT_YUYV16_1X16:
		mode = YCPOS_CrYCbY;
		break;
	case V4L2_MBUS_FMT_UYVY16_1X16:
		mode = YCPOS_YCrYCb;
		break;
	default:
		return;
	}

	pcr = isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR);
	pcr &= ~ISPPRV_PCR_YCPOS_CrYCbY;
	pcr |= (mode << ISPPRV_PCR_YCPOS_SHIFT);
	isp_reg_writel(isp, pcr, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR);
}

/**
 * isppreview_config_averager - Enable / disable / configure averager
 * @average: Average value to be configured.
 **/
static void
isppreview_config_averager(struct isp_prev_device *prev, u8 average)
{
	struct isp_device *isp = to_isp_device(prev);
	int reg = 0;

	if (prev->params.cfa.format == CFAFMT_BAYER)
		reg = ISPPRV_AVE_EVENDIST_2 << ISPPRV_AVE_EVENDIST_SHIFT |
		      ISPPRV_AVE_ODDDIST_2 << ISPPRV_AVE_ODDDIST_SHIFT |
		      average;
	else if (prev->params.cfa.format == CFAFMT_RGBFOVEON)
		reg = ISPPRV_AVE_EVENDIST_3 << ISPPRV_AVE_EVENDIST_SHIFT |
		      ISPPRV_AVE_ODDDIST_3 << ISPPRV_AVE_ODDDIST_SHIFT |
		      average;
	isp_reg_writel(isp, reg, OMAP3_ISP_IOMEM_PREV, ISPPRV_AVE);
}

/**
 * isppreview_set_input_sz - Set input frame size
 * @sph: Start pixel horizontal.
 * @eph: End pixel horizontal.
 * @slv: Start line vertical.
 * @elv: End line vertical.
 **/
static void isppreview_set_input_sz(struct isp_prev_device *prev,
				    u32 sph, u32 eph, u32 slv, u32 elv)
{
	struct isp_device *isp = to_isp_device(prev);

	isp_reg_writel(isp, (sph << ISPPRV_HORZ_INFO_SPH_SHIFT) | eph,
		       OMAP3_ISP_IOMEM_PREV, ISPPRV_HORZ_INFO);
	isp_reg_writel(isp, (slv << ISPPRV_VERT_INFO_SLV_SHIFT) | elv,
		       OMAP3_ISP_IOMEM_PREV, ISPPRV_VERT_INFO);
}

/*
 * isppreview_config_inlineoffset - Configures the Read address line offset.
 * @prev: Preview module
 * @offset: Line offset
 *
 * According to the TRM, the line offset must be aligned on a 32 bytes boundary.
 * However, a hardware bug requires the memory start address to be aligned on a
 * 64 bytes boundary, so the offset probably should be aligned on 64 bytes as
 * well.
 */
static void isppreview_config_inlineoffset(struct isp_prev_device *prev,
					   u32 offset)
{
	struct isp_device *isp = to_isp_device(prev);

	isp_reg_writel(isp, offset & 0xffff, OMAP3_ISP_IOMEM_PREV,
		       ISPPRV_RADR_OFFSET);
}

/*
 * isppreview_set_inaddr - Sets memory address of input frame.
 * @addr: 32bit memory address aligned on 32byte boundary.
 *
 * Configures the memory address from which the input frame is to be read.
 */
static void isppreview_set_inaddr(struct isp_prev_device *prev, u32 addr)
{
	struct isp_device *isp = to_isp_device(prev);

	isp_reg_writel(isp, addr, OMAP3_ISP_IOMEM_PREV, ISPPRV_RSDR_ADDR);
}

/*
 * isppreview_config_outlineoffset - Configures the Write address line offset.
 * @offset: Line Offset for the preview output.
 *
 * The offset must be a multiple of 32 bytes.
 */
static void isppreview_config_outlineoffset(struct isp_prev_device *prev,
				    u32 offset)
{
	struct isp_device *isp = to_isp_device(prev);

	isp_reg_writel(isp, offset & 0xffff, OMAP3_ISP_IOMEM_PREV,
		       ISPPRV_WADD_OFFSET);
}

/*
 * isppreview_set_outaddr - Sets the memory address to store output frame
 * @addr: 32bit memory address aligned on 32byte boundary.
 *
 * Configures the memory address to which the output frame is written.
 */
static void isppreview_set_outaddr(struct isp_prev_device *prev, u32 addr)
{
	struct isp_device *isp = to_isp_device(prev);

	isp_reg_writel(isp, addr, OMAP3_ISP_IOMEM_PREV, ISPPRV_WSDR_ADDR);
}

/**
 * isppreview_busy - Gets busy state of preview module.
 **/
int isppreview_busy(struct isp_prev_device *prev)
{
	struct isp_device *isp = to_isp_device(prev);

	return isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR)
		& ISPPRV_PCR_BUSY;
}

/**
 * isppreview_save_context - Saves the values of the preview module registers.
 **/
void isppreview_save_context(struct isp_device *isp)
{
	isp_save_context(isp, ispprev_reg_list);
	/* Avoid unwanted enabling when restoring the context. */
	ispprev_reg_list[0].val &= ~ISPPRV_PCR_EN;
}

/**
 * isppreview_restore_context - Restores the values of preview module registers
 **/
void isppreview_restore_context(struct isp_device *isp)
{
	isp_restore_context(isp, ispprev_reg_list);

	isp->isp_prev.update = PREV_FEATURES_END - 1;
	isppreview_setup_hw(&isp->isp_prev);
}

/**
 * isppreview_print_status - Prints the values of the Preview Module registers.
 *
 * Also prints other debug information stored in the preview moduel.
 **/
void isppreview_print_status(struct isp_prev_device *prev)
{
	struct isp_device *isp = to_isp_device(prev);

	dev_dbg(isp->dev, "###ISP_CTRL in preview =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL));
	dev_dbg(isp->dev, "###ISP_IRQ0ENABLE in preview =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE));
	dev_dbg(isp->dev, "###ISP_IRQ0STATUS in preview =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS));
	dev_dbg(isp->dev, "###PRV PCR =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR));
	dev_dbg(isp->dev, "###PRV HORZ_INFO =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_HORZ_INFO));
	dev_dbg(isp->dev, "###PRV VERT_INFO =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_VERT_INFO));
	dev_dbg(isp->dev, "###PRV WSDR_ADDR =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_WSDR_ADDR));
	dev_dbg(isp->dev, "###PRV WADD_OFFSET =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_WADD_OFFSET));
	dev_dbg(isp->dev, "###PRV AVE =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_AVE));
	dev_dbg(isp->dev, "###PRV HMED =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_HMED));
	dev_dbg(isp->dev, "###PRV NF =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_NF));
	dev_dbg(isp->dev, "###PRV WB_DGAIN =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_WB_DGAIN));
	dev_dbg(isp->dev, "###PRV WBGAIN =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_WBGAIN));
	dev_dbg(isp->dev, "###PRV WBSEL =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_WBSEL));
	dev_dbg(isp->dev, "###PRV CFA =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_CFA));
	dev_dbg(isp->dev, "###PRV BLKADJOFF =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_BLKADJOFF));
	dev_dbg(isp->dev, "###PRV RGB_MAT1 =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_RGB_MAT1));
	dev_dbg(isp->dev, "###PRV RGB_MAT2 =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_RGB_MAT2));
	dev_dbg(isp->dev, "###PRV RGB_MAT3 =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_RGB_MAT3));
	dev_dbg(isp->dev, "###PRV RGB_MAT4 =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_RGB_MAT4));
	dev_dbg(isp->dev, "###PRV RGB_MAT5 =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_RGB_MAT5));
	dev_dbg(isp->dev, "###PRV RGB_OFF1 =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_RGB_OFF1));
	dev_dbg(isp->dev, "###PRV RGB_OFF2 =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_RGB_OFF2));
	dev_dbg(isp->dev, "###PRV CSC0 =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_CSC0));
	dev_dbg(isp->dev, "###PRV CSC1 =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_CSC1));
	dev_dbg(isp->dev, "###PRV CSC2 =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_CSC2));
	dev_dbg(isp->dev, "###PRV CSC_OFFSET =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_CSC_OFFSET));
	dev_dbg(isp->dev, "###PRV CNT_BRT =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_CNT_BRT));
	dev_dbg(isp->dev, "###PRV CSUP =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_CSUP));
	dev_dbg(isp->dev, "###PRV SETUP_YC =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_SETUP_YC));
}

/**
 * isppreview_init_params - init image processing parameters.
 * @prev: pointer to previewer private structure
 * return none
 **/
static void isppreview_init_params(struct isp_prev_device *prev)
{
	struct isp_device *isp = to_isp_device(prev);
	struct prev_params *params = &prev->params;
	int i = 0;

	/* Init values */
	params->contrast = ISPPRV_CONTRAST_DEF * ISPPRV_CONTRAST_UNITS;
	params->brightness = ISPPRV_BRIGHT_DEF * ISPPRV_BRIGHT_UNITS;
	params->average = NO_AVE;
	params->lens_shading_shift = 0;
	params->cfa.format = CFAFMT_BAYER;
	memcpy(params->cfa.table, cfa_coef_table,
	       sizeof(params->cfa.table));
	params->cfa.gradthrs_horz = FLR_CFA_GRADTHRS_HORZ;
	params->cfa.gradthrs_vert = FLR_CFA_GRADTHRS_VERT;
	params->csup.gain = FLR_CSUP_GAIN;
	params->csup.thres = FLR_CSUP_THRES;
	params->csup.hypf_en = 0;
	memcpy(params->luma.table, luma_enhance_table,
	       sizeof(params->luma.table));
	params->nf.spread = FLR_NF_STRGTH;
	memcpy(params->nf.table, noise_filter_table, sizeof(params->nf.table));
	params->dcor.couplet_mode_en = 1;
	for (i = 0; i < ISPPRV_DETECT_CORRECT_CHANNELS; i++)
		params->dcor.detect_correct[i] = DEF_DETECT_CORRECT_VAL;
	memcpy(params->gamma.blue, bluegamma_table, sizeof(params->gamma.blue));
	memcpy(params->gamma.green, greengamma_table,
	       sizeof(params->gamma.green));
	memcpy(params->gamma.red, redgamma_table, sizeof(params->gamma.red));
	params->wbal.dgain = FLR_WBAL_DGAIN;
	if (isp->revision == ISP_REVISION_1_0) {
		params->wbal.coef0 = FLR_WBAL_COEF0_ES1;
		params->wbal.coef1 = FLR_WBAL_COEF1_ES1;
		params->wbal.coef2 = FLR_WBAL_COEF2_ES1;
		params->wbal.coef3 = FLR_WBAL_COEF3_ES1;
	} else {
		params->wbal.coef0 = FLR_WBAL_COEF0;
		params->wbal.coef1 = FLR_WBAL_COEF1;
		params->wbal.coef2 = FLR_WBAL_COEF2;
		params->wbal.coef3 = FLR_WBAL_COEF3;
	}
	params->blk_adj.red = FLR_BLKADJ_RED;
	params->blk_adj.green = FLR_BLKADJ_GREEN;
	params->blk_adj.blue = FLR_BLKADJ_BLUE;
	params->rgb2rgb = flr_rgb2rgb;
	params->rgb2ycbcr = flr_prev_csc;
	params->yclimit.minC = ISPPRV_YC_MIN;
	params->yclimit.maxC = ISPPRV_YC_MAX;
	params->yclimit.minY = ISPPRV_YC_MIN;
	params->yclimit.maxY = ISPPRV_YC_MAX;

	params->features = PREV_CFA | PREV_DEFECT_COR | PREV_NOISE_FILTER |
			   PREV_GAMMA | PREV_AVERAGER | PREV_BLKADJ |
			   PREV_YCLIMITS | PREV_RGB2RGB | PREV_COLOR_CONV |
			   PREV_WB | PREV_BRIGHTNESS | PREV_CONTRAST;

	params->features &= ~(PREV_AVERAGER | PREV_INVERSE_ALAW |
			      PREV_HORZ_MEDIAN_FILTER |
			      PREV_GAMMA_BYPASS |
			      PREV_DARK_FRAME_SUBTRACT |
			      PREV_LENS_SHADING |
			      PREV_DARK_FRAME_CAPTURE |
			      PREV_CHROMA_SUPPRESS |
			      PREV_LUMA_ENHANCE);

	isppreview_config_contrast(prev, params->contrast);
	isppreview_config_brightness(prev, params->brightness);

	prev->update = PREV_FEATURES_END - 1;

	isppreview_setup_hw(prev);

	/* Disable SDR PORT which is enabled by default after reset. */
	isp_reg_and(to_isp_device(prev), OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
		    ~ISPPRV_PCR_SDRPORT);
}

static void preview_enable_oneshot(struct isp_prev_device *prev)
{
	struct isp_device *isp = to_isp_device(prev);

	/* The PCR.SOURCE bit is automatically reset to 0 when the PCR.ENABLE
	 * bit is set. As the preview engine is used in single-shot mode, we
	 * need to set PCR.SOURCE before enabling the preview engine.
	 */
	if (prev->input == PREVIEW_INPUT_MEMORY)
		isp_reg_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
				   ISPPRV_PCR_SOURCE);

	isp_reg_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
		   ISPPRV_PCR_EN | ISPPRV_PCR_ONESHOT);
}

void isppreview_isr_frame_sync(struct isp_prev_device *prev)
{
	if (prev->state == ISP_PIPELINE_STREAM_CONTINUOUS && prev->underrun) {
		preview_enable_oneshot(prev);
		prev->underrun = 0;
	}
}

static void preview_isr_buffer(struct isp_prev_device *prev)
{
	struct isp_buffer *buffer;
	int restart = 0;

	if (prev->input == PREVIEW_INPUT_MEMORY) {
		buffer = isp_video_buffer_next(&prev->video_in, prev->error);
		if (buffer != NULL)
			isppreview_set_inaddr(prev, buffer->isp_addr);
	}

	if (prev->output & PREVIEW_OUTPUT_MEMORY) {
		buffer = isp_video_buffer_next(&prev->video_out, prev->error);
		if (buffer != NULL) {
			isppreview_set_outaddr(prev, buffer->isp_addr);
			restart = 1;
		}
	}

	if (prev->state == ISP_PIPELINE_STREAM_SINGLESHOT) {
		if (prev->output & PREVIEW_OUTPUT_MEMORY &&
		    isp_pipeline_ready(prev->video_out.pipe))
			isp_pipeline_set_stream(prev->video_out.isp,
						&prev->video_out,
						ISP_PIPELINE_STREAM_SINGLESHOT);
	} else {
		/* If an underrun occurs, the video queue operation handler will
		 * restart the preview engine. Otherwise restart it immediately.
		 */
		if (restart)
			preview_enable_oneshot(prev);
	}

	prev->error = 0;
}

/*
 * isppreview_isr - ISP preview engine interrupt handler
 *
 * Manage the preview engine video buffers and configure shadowed registers.
 */
void isppreview_isr(struct isp_prev_device *prev)
{
	unsigned long flags;

	spin_lock_irqsave(&prev->lock, flags);
	if (prev->shadow_update)
		goto done;

	if (prev->update & PREV_BRIGHTNESS) {
		isppreview_config_brightness(prev, prev->params.brightness);
		prev->update &= ~PREV_BRIGHTNESS;
	}

	if (prev->update & PREV_CONTRAST) {
		isppreview_config_contrast(prev, prev->params.contrast);
		prev->update &= ~PREV_CONTRAST;
	}

	isppreview_setup_hw(prev);

done:
	spin_unlock_irqrestore(&prev->lock, flags);

	if (prev->state == ISP_PIPELINE_STREAM_STOPPED)
		return;

	if (prev->input == PREVIEW_INPUT_MEMORY ||
	    prev->output & PREVIEW_OUTPUT_MEMORY)
		preview_isr_buffer(prev);
	else
		preview_enable_oneshot(prev);
}

/* -----------------------------------------------------------------------------
 * ISP video operations
 */

static int preview_video_queue(struct isp_video *video,
			       struct isp_buffer *buffer)
{
	struct isp_prev_device *prev = &video->isp->isp_prev;

	if (video->type == V4L2_BUF_TYPE_VIDEO_OUTPUT)
		isppreview_set_inaddr(prev, buffer->isp_addr);

	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		isppreview_set_outaddr(prev, buffer->isp_addr);

		/* We now have a buffer queued on the output, restart the
		 * pipeline on the next sync interrupt if running in continuous
		 * mode.
		 */
		if (prev->state == ISP_PIPELINE_STREAM_CONTINUOUS)
			prev->underrun = 1;
	}

	return 0;
}

static const struct isp_video_operations preview_video_ops = {
	.queue = preview_video_queue,
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev operations
 */

/* previewer format descriptions */
const static unsigned int prev_input_fmts[] = {
	V4L2_MBUS_FMT_SGRBG10_1X10,
};

const static unsigned int prev_output_fmts[] = {
	V4L2_MBUS_FMT_UYVY16_1X16,
	V4L2_MBUS_FMT_YUYV16_1X16,
};

/* Preview module controls */
static struct v4l2_queryctrl preview_controls[] = {
	{
		.id = V4L2_CID_BRIGHTNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Brightness",
		.minimum = ISPPRV_BRIGHT_LOW,
		.maximum = ISPPRV_BRIGHT_HIGH,
		.step = ISPPRV_BRIGHT_STEP,
		.default_value = ISPPRV_BRIGHT_DEF,
	},
	{
		.id = V4L2_CID_CONTRAST,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "Contrast",
		.minimum = ISPPRV_CONTRAST_LOW,
		.maximum = ISPPRV_CONTRAST_HIGH,
		.step = ISPPRV_CONTRAST_STEP,
		.default_value = ISPPRV_CONTRAST_DEF,
	},
};

/**
 * preview_g_ctrl - Handle get control subdev method
 * @sd  : pointer to v4l2 subdev structure
 * @ctrl: pointer to v4l2 control structure
 * return -EINVAL or zero on success
 **/
static int preview_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct isp_prev_device *prev = v4l2_get_subdevdata(sd);
	u8 current_value;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		isppreview_query_brightness(prev, &current_value);
		ctrl->value = current_value;
		break;
	case V4L2_CID_CONTRAST:
		isppreview_query_contrast(prev, &current_value);
		ctrl->value = current_value;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * preview_s_ctrl - Handle set control subdev method
 * @sd  : pointer to v4l2 subdev structure
 * @ctrl: pointer to v4l2 control structure
 * return -EINVAL or zero on success
 **/
static int preview_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct isp_prev_device *prev = v4l2_get_subdevdata(sd);
	u8 new_value = ctrl->value;
	int rval = 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
		rval = isppreview_update_brightness(prev, new_value);
		break;
	case V4L2_CID_CONTRAST:
		rval = isppreview_update_contrast(prev, new_value);
		break;
	default:
		rval = -EINVAL;
		break;
	}

	return rval;
}

/*
 * preview_query_ctrl - Handle query control subdev method
 * @sd  : pointer to v4l2 subdev structure
 * @ctrl: pointer to v4l2 control structure
 * return -EINVAL or zero on success
 */
static int
preview_query_ctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *query)
{
	struct v4l2_queryctrl *best = NULL;
	int next;
	u32 id;
	int i;

	next = query->id & V4L2_CTRL_FLAG_NEXT_CTRL;
	id = query->id & V4L2_CTRL_ID_MASK;

	for (i = 0; i < ARRAY_SIZE(preview_controls); i++) {
		struct v4l2_queryctrl *ctrl = &preview_controls[i];

		if (ctrl->id == id && !next) {
			best = ctrl;
			break;
		}

		if ((!best || best->id > ctrl->id) && ctrl->id > id && next)
			best = ctrl;
	}

	if (best == NULL)
		return -EINVAL;

	memcpy(query, best, sizeof(*query));
	return 0;
}

/**
 * preview_ioctl - Handle preview module private ioctl's
 * @prev: pointer to preview context structure
 * @cmd: configuration command
 * @arg: configuration argument
 * return -EINVAL or zero on success
 **/
static long preview_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct isp_prev_device *prev = v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_PRIVATE_ISP_PRV_CFG:
		return isppreview_config(prev, arg);

	default:
		return -ENOIOCTLCMD;
	}
}

/**
 * preview_s_power - Handle set power subdev method
 * @sd: pointer to v4l2 subdev structure
 * @on: power on/off
 * return -EINVAL or zero on success
 **/
static int preview_s_power(struct v4l2_subdev *sd, int on)
{
	struct isp_prev_device *prev = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(prev);

	if (on) {
		if (!isp_get(isp))
			return -EBUSY;

		isp_reg_or(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL,
			   ISPCTRL_PREV_RAM_EN | ISPCTRL_PREV_CLK_EN);
	} else {
		isp_reg_and(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL,
			    ~(ISPCTRL_PREV_CLK_EN | ISPCTRL_PREV_RAM_EN));
		isp_put(isp);
	}

	return 0;
}

/**
 * preview_s_stream - Enable/Disable streaming on preview subdev
 * @sd    : pointer to v4l2 subdev structure
 * @enable: 1 == Enable, 0 == Disable
 * return -EINVAL or zero on sucess
 **/
static int preview_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct isp_prev_device *prev = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(prev);

	switch (enable) {
	case ISP_PIPELINE_STREAM_CONTINUOUS:
		if (prev->output & PREVIEW_OUTPUT_MEMORY)
			isp_sbl_enable(isp, OMAP3_ISP_SBL_PREVIEW_WRITE);

		prev->underrun = 0;
		if (!(prev->output & PREVIEW_OUTPUT_MEMORY))
			preview_enable_oneshot(prev);
		break;

	case ISP_PIPELINE_STREAM_SINGLESHOT:
		if (prev->input == PREVIEW_INPUT_MEMORY)
			isp_sbl_enable(isp, OMAP3_ISP_SBL_PREVIEW_READ);
		if (prev->output & PREVIEW_OUTPUT_MEMORY)
			isp_sbl_enable(isp, OMAP3_ISP_SBL_PREVIEW_WRITE);

		preview_enable_oneshot(prev);
		break;

	case ISP_PIPELINE_STREAM_STOPPED:
		isp_sbl_disable(isp, OMAP3_ISP_SBL_PREVIEW_READ);
		isp_sbl_disable(isp, OMAP3_ISP_SBL_PREVIEW_WRITE);
		break;
	}

	prev->state = enable;
	return 0;
}

/*
 * preview_enum_mbus_code - Handle pixel format enumeration
 * @sd     : pointer to v4l2 subdev structure
 * @code   : pointer to v4l2_subdev_pad_mbus_code_enum structure
 * return -EINVAL or zero on success
 */
static int preview_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_mbus_code_enum *code)
{
	switch (code->pad) {
	case PREV_PAD_SINK:
		if (code->index >= ARRAY_SIZE(prev_input_fmts))
			return -EINVAL;

		code->code = prev_input_fmts[code->index];
		break;
	case PREV_PAD_SOURCE:
		if (code->index >= ARRAY_SIZE(prev_output_fmts))
			return -EINVAL;

		code->code = prev_output_fmts[code->index];
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static struct v4l2_mbus_framefmt *
__preview_get_format(struct isp_prev_device *prev, unsigned int pad,
		     enum v4l2_subdev_format which)
{
	if (which != V4L2_SUBDEV_FORMAT_PROBE &&
	    which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return NULL;

	if (pad >= PREV_PADS_NUM)
		return NULL;

	return &prev->formats[pad][which];
}

/**
 * preview_get_format - Handle get format by pads subdev method
 * @sd : pointer to v4l2 subdev structure
 * @pad: pad num
 * @fmt: pointer to v4l2 format structure
 * return -EINVAL or zero on sucess
 **/
static int preview_get_format(struct v4l2_subdev *sd, unsigned int pad,
			      struct v4l2_mbus_framefmt *fmt,
			      enum v4l2_subdev_format which)
{
	struct isp_prev_device *prev = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format =
		__preview_get_format(prev, pad, which);

	if (format == NULL)
		return -EINVAL;

	memcpy(fmt, format, sizeof(*fmt));
	return 0;
}

/*
 * __isppreview_out_limit - Handle previewer hardware ouput limitations
 * @isp_revision : ISP revision
 * returns maximum width output for current isp revision
 */
static inline unsigned int __isppreview_out_limit(unsigned int isp_revision)
{
	switch (isp_revision) {
	case ISP_REVISION_1_0:
		return ISPPRV_MAXOUTPUT_WIDTH;

	case ISP_REVISION_2_0:
	default:
		return ISPPRV_MAXOUTPUT_WIDTH_ES2;

	case ISP_REVISION_15_0:
		return ISPPRV_MAXOUTPUT_WIDTH_3630;
	}
}

/**
 * preview_try_format - Handle try format by pad subdev method
 * @sd : pointer to v4l2 subdev structure
 * @pad: pad num
 * @fmt: pointer to v4l2 format structure
 **/
static void preview_try_format(struct v4l2_subdev *sd, unsigned int pad,
			       struct v4l2_mbus_framefmt *fmt,
			       enum v4l2_subdev_format which)
{
	struct isp_prev_device *prev = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(prev);
	struct v4l2_mbus_framefmt *format;
	unsigned int max_out_width;
	enum v4l2_mbus_pixelcode pixelcode;

	max_out_width = __isppreview_out_limit(isp->revision);

	switch (pad) {
	case PREV_PAD_SINK:
		/* When reading data from the CCDC, the input size has already
		 * been mangled by the CCDC output pad so it can be accepted
		 * as-is.
		 *
		 * When reading data from memory, clamp the requested width and
		 * height. The TRM doesn't specify a minimum input height, make
		 * sure we got enough lines to enable the noise filter and color
		 * filter array interpolation.
		 */
		if (prev->input == PREVIEW_INPUT_MEMORY) {
			fmt->width = clamp_t(u32, fmt->width, PREV_MIN_WIDTH,
					     max_out_width * 8);
			fmt->height = clamp_t(u32, fmt->height, 8, 16384);
		}

		fmt->colorspace = V4L2_COLORSPACE_SRGB;
		fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;
		break;

	case PREV_PAD_SOURCE:
		pixelcode = fmt->code;
		format = __preview_get_format(prev, PREV_PAD_SINK, which);
		memcpy(fmt, format, sizeof(*fmt));

		/* The preview module output size is configurable through the
		 * input interface (horizontal and vertical cropping) and the
		 * averager (horizontal scaling by 1/1, 1/2, 1/4 or 1/8). In
		 * spite of this, hardcode the output size to the biggest
		 * possible value for simplicity reasons.
		 */
		switch (pixelcode) {
		case V4L2_MBUS_FMT_YUYV16_1X16:
		case V4L2_MBUS_FMT_UYVY16_1X16:
			fmt->code = pixelcode;
			break;

		default:
			fmt->code = V4L2_MBUS_FMT_YUYV16_1X16;
			break;
		}

		/* The TRM states (12.1.4.7.1.2) that 2 pixels must be cropped
		 * from the left and right sides when the input source is the
		 * CCDC. This seems not to be needed in practice, investigation
		 * is required.
		 */
		if (prev->input == PREVIEW_INPUT_CCDC)
			fmt->width -= 4;

		/* The preview module can output a maximum of 3312 pixels
		 * horizontally due to fixed memory-line sizes. Compute the
		 * horizontal averaging factor accordingly. Note that the limit
		 * applies to the noise filter and CFA interpolation blocks, so
		 * it doesn't take cropping by further blocks into account.
		 *
		 * ES 1.0 hardware revision is limited to 1280 pixels
		 * horizontally.
		 */
		fmt->width >>= fls(DIV_ROUND_UP(fmt->width, max_out_width) - 1);

		/* Assume that all blocks are enabled and crop pixels and lines
		 * accordingly.
		 *
		 * Median filter	4 pixels
		 * Noise filter		4 pixels, 4 lines
		 * CFA filter		4 pixels, 4 lines in Bayer mode
		 *				  2 lines in other modes
		 * Color suppression	2 pixels
		 * or luma enhancement
		 * -------------------------------------------------------------
		 * Maximum total	14 pixels, 8 lines
		 */
		fmt->width -= 14;
		fmt->height -= 8;

		fmt->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	}

	fmt->field = V4L2_FIELD_NONE;
}

/**
 * preview_set_format - Handle set format by pads subdev method
 * @sd : pointer to v4l2 subdev structure
 * @pad: pad num
 * @fmt: pointer to v4l2 format structure
 * return -EINVAL or zero on success
 **/
static int preview_set_format(struct v4l2_subdev *sd, unsigned int pad,
			      struct v4l2_mbus_framefmt *fmt,
			      enum v4l2_subdev_format which)
{
	struct isp_prev_device *prev = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(prev);
	struct v4l2_mbus_framefmt *format =
		__preview_get_format(prev, pad, which);
	unsigned int max_out_width;
	unsigned int fmt_avg;
	unsigned int width;

	if (format == NULL)
		return -EINVAL;

	preview_try_format(sd, pad, fmt, which);
	memcpy(format, fmt, sizeof(*format));

	if (which == V4L2_SUBDEV_FORMAT_PROBE)
		return 0;

	switch (pad) {
	case PREV_PAD_SINK:
		if (prev->input == PREVIEW_INPUT_CCDC) {
			isppreview_set_input_sz(prev, 2, fmt->width - 3,
						0, fmt->height - 1);
			isppreview_config_inlineoffset(prev, 0);
		} else {
			isppreview_set_input_sz(prev, 0, fmt->width - 1,
						0, fmt->height - 1);
			isppreview_config_inlineoffset(prev,
					ALIGN(fmt->width, 0x20) * 2);
		}
		break;

	case PREV_PAD_SOURCE:
		if (prev->output & PREVIEW_OUTPUT_MEMORY)
			isppreview_config_outlineoffset(prev,
					ALIGN(fmt->width, 0x10) * 2);

		max_out_width = __isppreview_out_limit(isp->revision);

		format = __preview_get_format(prev, PREV_PAD_SINK, which);
		width = format->width;
		fmt_avg = fls(DIV_ROUND_UP(width, max_out_width) - 1);
		isppreview_config_averager(prev, fmt_avg);
		isppreview_config_ycpos(prev, fmt->code);
		break;
	}

	return 0;
}

/* subdev core operations */
static const struct v4l2_subdev_core_ops preview_v4l2_core_ops = {
	.queryctrl = preview_query_ctrl,
	.g_ctrl = preview_g_ctrl,
	.s_ctrl = preview_s_ctrl,
	.ioctl = preview_ioctl,
	.s_power = preview_s_power,
};

/* subdev video operations */
static const struct v4l2_subdev_video_ops preview_v4l2_video_ops = {
	.s_stream = preview_s_stream,
};

/* subdev pad operations */
static const struct v4l2_subdev_pad_ops preview_v4l2_pad_ops = {
	.enum_mbus_code = preview_enum_mbus_code,
	.get_fmt = preview_get_format,
	.set_fmt = preview_set_format,
};

/* subdev operations */
static const struct v4l2_subdev_ops preview_v4l2_ops = {
	.core = &preview_v4l2_core_ops,
	.video = &preview_v4l2_video_ops,
	.pad = &preview_v4l2_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Media entity operations
 */

/**
 * preview_link_setup - Setup previewer connections.
 * @entity : Pointer to media entity structure
 * @local  : Pointer to local pad array
 * @remote : Pointer to remote pad array
 * @flags  : Link flags
 * return -EINVAL or zero on success
 **/
static int preview_link_setup(struct media_entity *entity,
			      const struct media_entity_pad *local,
			      const struct media_entity_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct isp_prev_device *prev = v4l2_get_subdevdata(sd);
	/* TODO: Must be removed */
	struct isp_device *isp = to_isp_device(prev);

	switch (local->index | (remote->entity->type << 16)) {
	case PREV_PAD_SINK | (MEDIA_ENTITY_TYPE_NODE << 16):
		/* read from memory */
		if (flags & MEDIA_LINK_FLAG_ACTIVE) {
			if (prev->input == PREVIEW_INPUT_CCDC)
				return -EBUSY;
			prev->input = PREVIEW_INPUT_MEMORY;
		} else {
			if (prev->input == PREVIEW_INPUT_MEMORY)
				prev->input = PREVIEW_INPUT_NONE;
		}
		break;

	case PREV_PAD_SINK | (MEDIA_ENTITY_TYPE_SUBDEV << 16):
		/* read from ccdc */
		if (flags & MEDIA_LINK_FLAG_ACTIVE) {
			if (prev->input == PREVIEW_INPUT_MEMORY)
				return -EBUSY;
			prev->input = PREVIEW_INPUT_CCDC;
		} else {
			if (prev->input == PREVIEW_INPUT_CCDC)
				prev->input = PREVIEW_INPUT_NONE;
		}
		break;

	case PREV_PAD_SOURCE | (MEDIA_ENTITY_TYPE_NODE << 16):
		/* write to memory */
		if (flags & MEDIA_LINK_FLAG_ACTIVE) {
			prev->output |= PREVIEW_OUTPUT_MEMORY;
			isp_reg_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
				   ISPPRV_PCR_SDRPORT);
		} else {
			prev->output &= ~PREVIEW_OUTPUT_MEMORY;
			isp_reg_and(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
				    ~ISPPRV_PCR_SDRPORT);
		}
		break;

	case PREV_PAD_SOURCE | (MEDIA_ENTITY_TYPE_SUBDEV << 16):
		/* write to resizer */
		if (flags & MEDIA_LINK_FLAG_ACTIVE) {
			prev->output |= PREVIEW_OUTPUT_RESIZER;
			isp_reg_or(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
				   ISPPRV_PCR_RSZPORT);
		} else {
			prev->output &= ~PREVIEW_OUTPUT_RESIZER;
			isp_reg_and(isp, OMAP3_ISP_IOMEM_PREV, ISPPRV_PCR,
				    ~ISPPRV_PCR_RSZPORT);
		}
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/* media operations */
static const struct media_entity_operations preview_media_ops = {
	.link_setup = preview_link_setup,
	.set_power = v4l2_subdev_set_power,
};

/**
 * isppreview_init_entities - Initialize subdev and media entity.
 * @prev : Pointer to isppreview structure
 * return -ENOMEM or zero on success
 **/
static int isppreview_init_entities(struct isp_prev_device *prev)
{
	struct v4l2_subdev *sd = &prev->subdev;
	struct media_entity_pad *pads = prev->pads;
	struct media_entity *me = &sd->entity;
	int ret;

	prev->input = PREVIEW_INPUT_NONE;

	v4l2_subdev_init(sd, &preview_v4l2_ops);
	strlcpy(sd->name, "OMAP3 ISP preview", sizeof(sd->name));
	sd->grp_id = 1 << 16;	/* group ID for isp subdevs */
	v4l2_set_subdevdata(sd, prev);

	pads[PREV_PAD_SINK].type = MEDIA_PAD_TYPE_INPUT;
	pads[PREV_PAD_SOURCE].type = MEDIA_PAD_TYPE_OUTPUT;

	me->ops = &preview_media_ops;
	ret = media_entity_init(me, PREV_PADS_NUM, pads, 0);
	if (ret < 0)
		return ret;

	/* According to the OMAP34xx TRM, video buffers need to be aligned on a
	 * 32 bytes boundary. However, an undocumented hardware bug requires a
	 * 64 bytes boundary at the preview engine input.
	 */
	prev->video_in.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	prev->video_in.ops = &preview_video_ops;
	prev->video_in.isp = to_isp_device(prev);
	prev->video_in.capture_mem = PAGE_ALIGN(4096 * 4096) * 2 * 3;
	prev->video_in.alignment = 64;
	prev->video_out.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	prev->video_out.ops = &preview_video_ops;
	prev->video_out.isp = to_isp_device(prev);
	prev->video_out.capture_mem = PAGE_ALIGN(4096 * 4096) * 2 * 3;
	prev->video_out.alignment = 32;

	ret = isp_video_init(&prev->video_in, "preview");
	if (ret < 0)
		return ret;

	ret = isp_video_init(&prev->video_out, "preview");
	if (ret < 0)
		return ret;

	/* Connect the video nodes to the previewer subdev. */
	ret = media_entity_create_link(&prev->video_in.video.entity, 0,
			&prev->subdev.entity, PREV_PAD_SINK, 0);
	if (ret < 0)
		return ret;

	ret = media_entity_create_link(&prev->subdev.entity, PREV_PAD_SOURCE,
			&prev->video_out.video.entity, 0, 0);
	if (ret < 0)
		return ret;

	return 0;
}

void isp_preview_unregister_entities(struct isp_prev_device *prev)
{
	media_entity_cleanup(&prev->subdev.entity);

	v4l2_device_unregister_subdev(&prev->subdev);
	isp_video_unregister(&prev->video_in);
	isp_video_unregister(&prev->video_out);
}

int isp_preview_register_entities(struct isp_prev_device *prev,
	struct v4l2_device *vdev)
{
	int ret;

	/* Register the subdev and video nodes. */
	ret = v4l2_device_register_subdev(vdev, &prev->subdev);
	if (ret < 0)
		goto error;

	ret = isp_video_register(&prev->video_in, vdev);
	if (ret < 0)
		goto error;

	ret = isp_video_register(&prev->video_out, vdev);
	if (ret < 0)
		goto error;

	return 0;

error:
	isp_preview_unregister_entities(prev);
	return ret;
}

/* -----------------------------------------------------------------------------
 * ISP previewer initialisation and cleanup
 */

void isp_preview_cleanup(struct isp_device *isp)
{
}

/**
 * isp_preview_init - Previewer initialization.
 * @dev : Pointer to ISP device
 * return -ENOMEM or zero on success
 **/
int isp_preview_init(struct isp_device *isp)
{
	struct isp_prev_device *prev = &isp->isp_prev;
	int ret;

	spin_lock_init(&prev->lock);
	isppreview_init_params(prev);

	ret = isppreview_init_entities(prev);
	if (ret < 0)
		goto out;

out:
	if (ret)
		isp_preview_cleanup(isp);

	return ret;
}

