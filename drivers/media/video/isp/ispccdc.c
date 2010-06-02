/*
 * ispccdc.c
 *
 * Driver Library for CCDC module in TI's OMAP3 Camera ISP
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

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/sched.h>
#include <media/v4l2-event.h>

#include "isp.h"
#include "ispreg.h"
#include "ispccdc.h"

#define LSC_TABLE_INIT_SIZE	50052

static struct v4l2_mbus_framefmt *
__ccdc_get_format(struct isp_ccdc_device *ccdc, unsigned int pad,
		  enum v4l2_subdev_format which);

/* Structure for saving/restoring CCDC module registers*/
static struct isp_reg ispccdc_reg_list[] = {
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SYN_MODE, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HD_VD_WID, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PIX_LINES, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HORZ_INFO, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VERT_START, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VERT_LINES, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CULLING, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HSIZE_OFF, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDR_ADDR, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CLAMP, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_DCSUB, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_COLPTN, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_BLKCMP, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC_ADDR, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VDINT, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_ALAW, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_REC656IF, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CFG, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMTCFG, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_HORZ, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_VERT, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR0, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR1, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR2, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR3, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR4, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR5, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR6, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_ADDR7, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGEVEN0, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGEVEN1, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGODD0, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PRGODD1, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VP_OUT, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_CONFIG, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_INITIAL, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_TABLE_BASE, 0},
	{OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_TABLE_OFFSET, 0},
	{0, ISP_TOK_TERM, 0}
};

/*
 * ispccdc_save_context - Save values of the CCDC module registers
 * @dev: Device pointer specific to the OMAP3 ISP.
 */
void ispccdc_save_context(struct isp_device *isp)
{
	isp_save_context(isp, ispccdc_reg_list);
}

/*
 * ispccdc_restore_context - Restore values of the CCDC module registers
 * @dev: Pointer to ISP device
 */
void ispccdc_restore_context(struct isp_device *isp)
{
	isp_restore_context(isp, ispccdc_reg_list);
}

/*
 * ispccdc_print_status - Print current CCDC Module register values.
 * @ccdc: Pointer to ISP CCDC device.
 *
 * Also prints other debug information stored in the CCDC module.
 **/
static void ispccdc_print_status(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);

	dev_dbg(isp->dev, "###CCDC PCR=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PCR));
	dev_dbg(isp->dev, "ISP_CTRL =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL));
	dev_dbg(isp->dev, "ccdc data format is %d\n",
		ccdc->formats[CCDC_PAD_SINK][V4L2_SUBDEV_FORMAT_ACTIVE].code);
	dev_dbg(isp->dev, "###ISP_CTRL in ccdc =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL));
	dev_dbg(isp->dev, "###ISP_IRQ0ENABLE in ccdc =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE));
	dev_dbg(isp->dev, "###ISP_IRQ0STATUS in ccdc =0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS));
	dev_dbg(isp->dev, "###CCDC SYN_MODE=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SYN_MODE));
	dev_dbg(isp->dev, "###CCDC HORZ_INFO=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HORZ_INFO));
	dev_dbg(isp->dev, "###CCDC VERT_START=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VERT_START));
	dev_dbg(isp->dev, "###CCDC VERT_LINES=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VERT_LINES));
	dev_dbg(isp->dev, "###CCDC CULLING=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CULLING));
	dev_dbg(isp->dev, "###CCDC HSIZE_OFF=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HSIZE_OFF));
	dev_dbg(isp->dev, "###CCDC SDOFST=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST));
	dev_dbg(isp->dev, "###CCDC SDR_ADDR=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDR_ADDR));
	dev_dbg(isp->dev, "###CCDC CLAMP=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CLAMP));
	dev_dbg(isp->dev, "###CCDC COLPTN=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_COLPTN));
	dev_dbg(isp->dev, "###CCDC CFG=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CFG));
	dev_dbg(isp->dev, "###CCDC VP_OUT=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VP_OUT));
	dev_dbg(isp->dev, "###CCDC_SDR_ADDR= 0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDR_ADDR));
	dev_dbg(isp->dev, "###CCDC FMTCFG=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMTCFG));
	dev_dbg(isp->dev, "###CCDC FMT_HORZ=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_HORZ));
	dev_dbg(isp->dev, "###CCDC FMT_VERT=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_VERT));
	dev_dbg(isp->dev, "###CCDC LSC_CONFIG=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_CONFIG));
	dev_dbg(isp->dev, "###CCDC LSC_INIT=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_INITIAL));
	dev_dbg(isp->dev, "###CCDC LSC_TABLE BASE=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC,
		ISPCCDC_LSC_TABLE_BASE));
	dev_dbg(isp->dev, "###CCDC LSC TABLE OFFSET=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC,
		ISPCCDC_LSC_TABLE_OFFSET));
}

/*
 * ispccdc_busy - Get busy state of the CCDC.
 * @ccdc: Pointer to ISP CCDC device.
 */
int ispccdc_busy(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);

	return isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PCR) &
		ISPCCDC_PCR_BUSY;
}

/* -----------------------------------------------------------------------------
 * Lens Shading Compensation
 */

/*
 * ispccdc_validate_config_lsc - Check that LSC configuration is valid.
 * @ccdc: Pointer to ISP CCDC device.
 * @lsc_cfg: the LSC configuration to check.
 *
 * Returns 0 if the LSC configuration is valid, or -EINVAL if invalid.
 */
static int ispccdc_validate_config_lsc(struct isp_ccdc_device *ccdc,
				       struct ispccdc_lsc_config *lsc_cfg)
{
	struct isp_device *isp = to_isp_device(ccdc);
	struct v4l2_mbus_framefmt *format;
	unsigned int paxel_width, paxel_height;
	unsigned int paxel_shift_x, paxel_shift_y;
	unsigned int min_width, min_height, min_size;
	unsigned int input_width, input_height;

	paxel_shift_x = lsc_cfg->gain_mode_m;
	paxel_shift_y = lsc_cfg->gain_mode_n;

	if ((paxel_shift_x < 2) || (paxel_shift_x > 6) ||
	    (paxel_shift_y < 2) || (paxel_shift_y > 6)) {
		dev_dbg(isp->dev, "CCDC: LSC: Invalid paxel size\n");
		return -EINVAL;
	}

	if (lsc_cfg->offset & 3) {
		dev_dbg(isp->dev, "CCDC: LSC: Offset must be a multiple of "
			"4\n");
		return -EINVAL;
	}

	if ((lsc_cfg->initial_x & 1) || (lsc_cfg->initial_y & 1)) {
		dev_dbg(isp->dev, "CCDC: LSC: initial_x and y must be even\n");
		return -EINVAL;
	}

	format = __ccdc_get_format(ccdc, CCDC_PAD_SINK,
				   V4L2_SUBDEV_FORMAT_ACTIVE);
	input_width = format->width;
	input_height = format->height;

	/* Calculate minimum bytesize for validation */
	paxel_width = 1 << paxel_shift_x;
	min_width = ((input_width + lsc_cfg->initial_x + paxel_width - 1)
		     >> paxel_shift_x) + 1;

	paxel_height = 1 << paxel_shift_y;
	min_height = ((input_height + lsc_cfg->initial_y + paxel_height - 1)
		     >> paxel_shift_y) + 1;

	min_size = 4 * min_width * min_height;
	if (min_size > lsc_cfg->size) {
		dev_dbg(isp->dev, "CCDC: LSC: too small table\n");
		return -EINVAL;
	}
	if (lsc_cfg->offset < (min_width * 4)) {
		dev_dbg(isp->dev, "CCDC: LSC: Offset is too small\n");
		return -EINVAL;
	}
	if ((lsc_cfg->size / lsc_cfg->offset) < min_height) {
		dev_dbg(isp->dev, "CCDC: LSC: Wrong size/offset combination\n");
		return -EINVAL;
	}
	return 0;
}

/*
 * ispccdc_program_lsc_table - Program Lens Shading Compensation table address.
 * @ccdc: Pointer to ISP CCDC device.
 */
static void ispccdc_program_lsc_table(struct isp_ccdc_device *ccdc)
{
	isp_reg_writel(to_isp_device(ccdc), ccdc->lsc.table_inuse,
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_TABLE_BASE);
}

/*
 * ispccdc_setup_lsc_regs - Configures the lens shading compensation module
 * @ccdc: Pointer to ISP CCDC device.
 */
static void ispccdc_setup_lsc_regs(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);
	struct ispccdc_lsc_config *lsc_cfg = &ccdc->lsc.config;
	int reg;

	isp_reg_writel(isp, lsc_cfg->offset, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_LSC_TABLE_OFFSET);

	reg = 0;
	reg |= lsc_cfg->gain_mode_n << ISPCCDC_LSC_GAIN_MODE_N_SHIFT;
	reg |= lsc_cfg->gain_mode_m << ISPCCDC_LSC_GAIN_MODE_M_SHIFT;
	reg |= lsc_cfg->gain_format << ISPCCDC_LSC_GAIN_FORMAT_SHIFT;
	isp_reg_writel(isp, reg, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_LSC_CONFIG);

	reg = 0;
	reg &= ~ISPCCDC_LSC_INITIAL_X_MASK;
	reg |= lsc_cfg->initial_x << ISPCCDC_LSC_INITIAL_X_SHIFT;
	reg &= ~ISPCCDC_LSC_INITIAL_Y_MASK;
	reg |= lsc_cfg->initial_y << ISPCCDC_LSC_INITIAL_Y_SHIFT;
	isp_reg_writel(isp, reg, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_LSC_INITIAL);
}

/*
 * ispccdc_enable_lsc - Enables/Disables the Lens Shading Compensation module.
 * @ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables LSC, 1 Enables LSC.
 */
static void ispccdc_enable_lsc(struct isp_ccdc_device *ccdc, u8 enable)
{
	struct isp_device *isp = to_isp_device(ccdc);
	const struct v4l2_mbus_framefmt *format =
		__ccdc_get_format(ccdc, CCDC_PAD_SINK,
				  V4L2_SUBDEV_FORMAT_ACTIVE);

	if (enable && (format->code == V4L2_MBUS_FMT_SGRBG10_1X10 ||
	     format->code == V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8)) {
		isp_sbl_enable(isp, OMAP3_ISP_SBL_CCDC_LSC_READ);
		isp_reg_or(isp, OMAP3_ISP_IOMEM_CCDC,
			   ISPCCDC_LSC_CONFIG, ISPCCDC_LSC_ENABLE);
	} else {
		isp_sbl_disable(isp, OMAP3_ISP_SBL_CCDC_LSC_READ);
		isp_reg_and(isp, OMAP3_ISP_IOMEM_CCDC,
			    ISPCCDC_LSC_CONFIG, ~ISPCCDC_LSC_ENABLE);
	}
}

/*
 * ispccdc_setup_lsc - apply user LSC settings
 * @ccdc: Pointer to ISP CCDC device.
 *
 * Consume the new LSC configuration and table set by user space application and
 * program to CCDC. This function must be called from process context before
 * streamon when the CCDC module is not yet running. This function does not yet
 * actually enable LSC, that has to be done separately.
 */
static void ispccdc_setup_lsc(struct isp_ccdc_device *ccdc)
{
	unsigned long flags;
	u32 lsc_old_table = 0;

	ispccdc_enable_lsc(ccdc, 0);	/* Disable LSC */
	spin_lock_irqsave(&ccdc->lock, flags);
	if (ccdc->lsc.request_enable) {
		/* LSC is requested to be enabled, so configure it */
		if (ccdc->lsc.update_table) {
			lsc_old_table = ccdc->lsc.table_inuse;
			ccdc->lsc.table_inuse = ccdc->lsc.table_new;
			ccdc->lsc.table_new = 0;
			ccdc->lsc.update_table = 0;
		}
		ispccdc_setup_lsc_regs(ccdc);
		ispccdc_program_lsc_table(ccdc);
	}
	ccdc->lsc.update_config = 0;
	spin_unlock_irqrestore(&ccdc->lock, flags);

	if (lsc_old_table != 0) {
		struct isp_device *isp = to_isp_device(ccdc);
		iommu_vfree(isp->iommu, lsc_old_table);
	}
}

/**
 * ispccdc_lsc_error_handler - Handle LSC prefetch error scenario.
 * @ccdc: Pointer to ISP CCDC device.
 *
 * Disables LSC, and defers enablement to shadow registers update time.
 **/
void ispccdc_lsc_error_handler(struct isp_ccdc_device *ccdc)
{
	/*
	 * From OMAP3 TRM: When this event is pending, the module
	 * goes into transparent mode (output =input). Normal
	 * operation can be resumed at the start of the next frame
	 * after:
	 *  1) Clearing this event
	 *  2) Disabling the LSC module
	 *  3) Enabling it
	 */
	ispccdc_enable_lsc(ccdc, 0);
	ispccdc_enable_lsc(ccdc, 1);
}

/* -----------------------------------------------------------------------------
 * Parameters configuration
 */

/*
 * ispccdc_config_black_clamp - Configures the clamp parameters in CCDC.
 * @ccdc: Pointer to ISP CCDC device.
 * @bclamp: Structure containing the optical black average gain, optical black
 *          sample length, sample lines, and the start pixel position of the
 *          samples w.r.t the HS pulse.
 *
 * Configures the clamp parameters in CCDC. Either if its being used the
 * optical black clamp, or the digital clamp. If its a digital clamp, then
 * assures to put a valid DC substraction level.
 *
 * Returns always 0 when completed.
 */
static int ispccdc_config_black_clamp(struct isp_ccdc_device *ccdc,
				      struct ispccdc_bclamp bclamp)
{
	struct isp_device *isp = to_isp_device(ccdc);
	u32 bclamp_val = 0;

	if (ccdc->obclamp_en) {
		bclamp_val |= bclamp.obgain << ISPCCDC_CLAMP_OBGAIN_SHIFT;
		bclamp_val |= bclamp.oblen << ISPCCDC_CLAMP_OBSLEN_SHIFT;
		bclamp_val |= bclamp.oblines << ISPCCDC_CLAMP_OBSLN_SHIFT;
		bclamp_val |= bclamp.obstpixel << ISPCCDC_CLAMP_OBST_SHIFT;
		isp_reg_writel(isp, bclamp_val,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CLAMP);
	} else {
		if (omap_rev() < OMAP3430_REV_ES2_0)
			if (ccdc->syncif_ipmod == YUV16 ||
			    ccdc->syncif_ipmod == YUV8 ||
			    isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC,
					  ISPCCDC_REC656IF) &
			    ISPCCDC_REC656IF_R656ON)
				bclamp.dcsubval = 0;
		isp_reg_writel(isp, bclamp.dcsubval,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_DCSUB);
	}
	return 0;
}

/*
 * ispccdc_enable_black_clamp - Enables/Disables the optical black clamp.
 * @ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables optical black clamp, 1 Enables optical black clamp.
 *
 * Enables or disables the optical black clamp. When disabled, the digital
 * clamp operates.
 */
static void ispccdc_enable_black_clamp(struct isp_ccdc_device *ccdc,
				       u8 enable)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CLAMP,
		       ~ISPCCDC_CLAMP_CLAMPEN,
		       enable ? ISPCCDC_CLAMP_CLAMPEN : 0);
	ccdc->obclamp_en = enable;
}

/*
 * ispccdc_config_fpc - Configures the Faulty Pixel Correction parameters.
 * @ccdc: Pointer to ISP CCDC device.
 * @fpc: Structure containing the number of faulty pixels corrected in the
 *       frame, address of the FPC table.
 *
 * Returns 0 if successful, or -EINVAL if FPC Address is not on the 64 byte
 * boundary.
 */
static int ispccdc_config_fpc(struct isp_ccdc_device *ccdc,
			      struct ispccdc_fpc fpc)
{
	struct isp_device *isp = to_isp_device(ccdc);
	u32 fpc_val = 0;

	fpc_val = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC);

	isp_reg_writel(isp, fpc_val & (~ISPCCDC_FPC_FPCEN),
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC);
	isp_reg_writel(isp, fpc.fpcaddr,
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC_ADDR);
	isp_reg_writel(isp, fpc_val | (fpc.fpnum << ISPCCDC_FPC_FPNUM_SHIFT),
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC);
	return 0;
}

/*
 * ispccdc_enable_fpc - Enable Faulty Pixel Correction.
 * @ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables FPC, 1 Enables FPC.
 */
static void ispccdc_enable_fpc(struct isp_ccdc_device *ccdc, u8 enable)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FPC,
		       ~ISPCCDC_FPC_FPCEN, enable ? ISPCCDC_FPC_FPCEN : 0);
}

/*
 * ispccdc_config_black_comp - Configure Black Level Compensation.
 * @ccdc: Pointer to ISP CCDC device.
 * @blcomp: Structure containing the black level compensation value for RGrGbB
 *          pixels. in 2's complement.
 */
static void ispccdc_config_black_comp(struct isp_ccdc_device *ccdc,
				      struct ispccdc_blcomp blcomp)
{
	struct isp_device *isp = to_isp_device(ccdc);
	u32 blcomp_val = 0;

	blcomp_val |= blcomp.b_mg << ISPCCDC_BLKCMP_B_MG_SHIFT;
	blcomp_val |= blcomp.gb_g << ISPCCDC_BLKCMP_GB_G_SHIFT;
	blcomp_val |= blcomp.gr_cy << ISPCCDC_BLKCMP_GR_CY_SHIFT;
	blcomp_val |= blcomp.r_ye << ISPCCDC_BLKCMP_R_YE_SHIFT;

	isp_reg_writel(isp, blcomp_val, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_BLKCMP);
}

/*
 * ispccdc_config_culling - Configure culling parameters.
 * @ccdc: Pointer to ISP CCDC device.
 * @cull: Structure containing the vertical culling pattern, and horizontal
 *        culling pattern for odd and even lines.
 */
static void ispccdc_config_culling(struct isp_ccdc_device *ccdc,
				   struct ispccdc_culling cull)
{
	struct isp_device *isp = to_isp_device(ccdc);

	u32 culling_val = 0;

	culling_val |= cull.v_pattern << ISPCCDC_CULLING_CULV_SHIFT;
	culling_val |= cull.h_even << ISPCCDC_CULLING_CULHEVN_SHIFT;
	culling_val |= cull.h_odd << ISPCCDC_CULLING_CULHODD_SHIFT;

	isp_reg_writel(isp, culling_val, OMAP3_ISP_IOMEM_CCDC,
		       ISPCCDC_CULLING);
}

/*
 * ispccdc_enable_lpf - Enable Low-Pass Filter (LPF).
 * @ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables LPF, 1 Enables LPF
 */
static void ispccdc_enable_lpf(struct isp_ccdc_device *ccdc, u8 enable)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SYN_MODE,
		       ~ISPCCDC_SYN_MODE_LPF,
		       enable ? ISPCCDC_SYN_MODE_LPF : 0);
}

/*
 * ispccdc_config_alaw - Configure the input width for A-law compression.
 * @ccdc: Pointer to ISP CCDC device.
 * @ipwidth: Input width for A-law
 */
static void ispccdc_config_alaw(struct isp_ccdc_device *ccdc,
				enum alaw_ipwidth ipwidth)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_writel(isp, ipwidth << ISPCCDC_ALAW_GWDI_SHIFT,
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_ALAW);
}

/*
 * ispccdc_enable_alaw - Enable A-law compression.
 * @ccdc: Pointer to ISP CCDC device.
 * @enable: 0 - Disables A-law, 1 - Enables A-law
 */
static void ispccdc_enable_alaw(struct isp_ccdc_device *ccdc, u8 enable)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_ALAW,
		       ~ISPCCDC_ALAW_CCDTBL,
		       enable ? ISPCCDC_ALAW_CCDTBL : 0);
}

/*
 * ispccdc_config_imgattr - Configure sensor image specific attributes.
 * @ccdc: Pointer to ISP CCDC device.
 * @colptn: Color pattern of the sensor.
 */
static void ispccdc_config_imgattr(struct isp_ccdc_device *ccdc, u32 colptn)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_writel(isp, colptn, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_COLPTN);
}

static void ispccdc_free_lsc_table_work(struct work_struct *work)
{
	struct ispccdc_lsc *lsc = container_of(work, struct ispccdc_lsc,
					       table_work);
	struct isp_ccdc_device *ccdc = container_of(lsc, struct isp_ccdc_device,
						    lsc);
	unsigned long flags;

	spin_lock_irqsave(&ccdc->lock, flags);
	if (lsc->table_old != 0) {
		struct isp_device *isp = to_isp_device(ccdc);
		u32 lsc_table_old = lsc->table_old;
		lsc->table_old = 0;
		spin_unlock_irqrestore(&ccdc->lock, flags);
		iommu_vfree(isp->iommu, lsc_table_old);
	} else {
		spin_unlock_irqrestore(&ccdc->lock, flags);
	}
}

static int ispccdc_config_lsc(struct isp_ccdc_device *ccdc,
			      struct ispccdc_update_config *ccdc_struct)
{
	u32 lsc_table_new = 0;
	u32 lsc_table_old = 0;
	int lsc_update_table = 0;
	int lsc_update_config = 0;
	struct ispccdc_lsc_config lsc_cfg;
	unsigned long flags;
	struct isp_device *isp = to_isp_device(ccdc);

	if (ISP_ABS_CCDC_CONFIG_LSC & ccdc_struct->update) {
		if (ISP_ABS_CCDC_CONFIG_LSC & ccdc_struct->flag) {
			int ret;

			if (copy_from_user(&lsc_cfg, ccdc_struct->lsc_cfg,
					   sizeof(lsc_cfg)))
				return -EFAULT;
			ret = ispccdc_validate_config_lsc(ccdc, &lsc_cfg);
			if (ret)
				return ret;
		}
		lsc_update_config = 1;
	}

	if (ISP_ABS_TBL_LSC & ccdc_struct->update) {
		void *n;
		u32 table_size;

		if (ISP_ABS_CCDC_CONFIG_LSC & ccdc_struct->update) {
			/* Using new config */
			table_size = lsc_cfg.size;
		} else {
			/*
			 * Using last valid config as LSC is not being
			 * configured at this moment.
			 */
			spin_lock_irqsave(&ccdc->lock, flags);
			table_size = ccdc->lsc.config.size;
			spin_unlock_irqrestore(&ccdc->lock, flags);
		}

		lsc_table_new = iommu_vmalloc(isp->iommu, 0, table_size,
					      IOMMU_FLAG);
		if (IS_ERR_VALUE(lsc_table_new))
			return -ENOMEM;
		n = da_to_va(isp->iommu, lsc_table_new);
		if (copy_from_user(n, ccdc_struct->lsc, table_size)) {
			iommu_vfree(isp->iommu, lsc_table_new);
			return -EFAULT;
		}
		lsc_update_table = 1;
	}

	/* Save LSC configuration */
	spin_lock_irqsave(&ccdc->lock, flags);
	if (lsc_update_config) {
		if (ISP_ABS_CCDC_CONFIG_LSC & ccdc_struct->flag) {
			memcpy(&ccdc->lsc.config, &lsc_cfg,
			       sizeof(ccdc->lsc.config));
			ccdc->lsc.request_enable = 1;
		} else {
			ccdc->lsc.request_enable = 0;
		}
		ccdc->lsc.update_config = 1;
	}
	if (lsc_update_table) {
		u32 temp_table = ccdc->lsc.table_new;

		ccdc->lsc.table_new = lsc_table_new;
		ccdc->lsc.update_table = 1;

		/* Save unused new table to deallocate it */
		lsc_table_new = temp_table;

		/*
		 * Any pending old table must be freed now. After the next ISR,
		 * a new old table will exist and the driver can deal with only
		 * one at once.
		 */
		lsc_table_old = ccdc->lsc.table_old;
		ccdc->lsc.table_old = 0;
	}
	spin_unlock_irqrestore(&ccdc->lock, flags);

	if (ccdc->state == ISP_PIPELINE_STREAM_STOPPED &&
	    (ccdc->lsc.update_table || ccdc->lsc.update_config))
		ispccdc_setup_lsc(ccdc);

	if (lsc_table_new != 0)
		iommu_vfree(isp->iommu, lsc_table_new);
	if (lsc_table_old != 0)
		iommu_vfree(isp->iommu, lsc_table_old);

	return 0;
}


/*
 * ispccdc_config - Set CCDC configuration from userspace
 * @ccdc: Pointer to ISP CCDC device.
 * @userspace_add: Structure containing CCDC configuration sent from userspace.
 *
 * Returns 0 if successful, -EINVAL if the pointer to the configuration
 * structure is null, or the copy_from_user function fails to copy user space
 * memory to kernel space memory.
 */
static int ispccdc_config(struct isp_ccdc_device *ccdc,
			  struct ispccdc_update_config *ccdc_struct)
{
	struct isp_device *isp = to_isp_device(ccdc);
	struct ispccdc_bclamp bclamp_t;
	struct ispccdc_blcomp blcomp_t;
	struct ispccdc_culling cull_t;
	unsigned long flags;
	int ret = 0;

	if (ccdc_struct == NULL)
		return -EINVAL;

	spin_lock_irqsave(&ccdc->lock, flags);
	ccdc->shadow_update = 1;
	spin_unlock_irqrestore(&ccdc->lock, flags);

	if (ISP_ABS_CCDC_ALAW & ccdc_struct->flag) {
		if (ISP_ABS_CCDC_ALAW & ccdc_struct->update)
			ispccdc_config_alaw(ccdc, ccdc_struct->alawip);
		ispccdc_enable_alaw(ccdc, 1);
	} else if (ISP_ABS_CCDC_ALAW & ccdc_struct->update)
		ispccdc_enable_alaw(ccdc, 0);

	if (ISP_ABS_CCDC_LPF & ccdc_struct->flag)
		ispccdc_enable_lpf(ccdc, 1);
	else
		ispccdc_enable_lpf(ccdc, 0);

	if (ISP_ABS_CCDC_BLCLAMP & ccdc_struct->flag) {
		if (ISP_ABS_CCDC_BLCLAMP & ccdc_struct->update) {
			if (copy_from_user(&bclamp_t, ccdc_struct->bclamp,
					   sizeof(struct ispccdc_bclamp))) {
				ret = -EFAULT;
				goto out;
			}

			ispccdc_enable_black_clamp(ccdc, 1);
			ispccdc_config_black_clamp(ccdc, bclamp_t);
		} else
			ispccdc_enable_black_clamp(ccdc, 1);
	} else {
		if (ISP_ABS_CCDC_BLCLAMP & ccdc_struct->update) {
			if (copy_from_user(&bclamp_t, ccdc_struct->bclamp,
					   sizeof(struct ispccdc_bclamp))) {
				ret = -EFAULT;
				goto out;
			}

			ispccdc_enable_black_clamp(ccdc, 0);
			ispccdc_config_black_clamp(ccdc, bclamp_t);
		}
	}

	if (ISP_ABS_CCDC_BCOMP & ccdc_struct->update) {
		if (copy_from_user(&blcomp_t, ccdc_struct->blcomp,
				   sizeof(blcomp_t))) {
				ret = -EFAULT;
				goto out;
			}

		ispccdc_config_black_comp(ccdc, blcomp_t);
	}

	if (ISP_ABS_CCDC_FPC & ccdc_struct->flag) {
		if (ISP_ABS_CCDC_FPC & ccdc_struct->update) {
			struct ispccdc_fpc fpc_t;
			u32 fpc_table_m;
			void *fpc_table;
			u32 fpc_table_old;
			u32 fpc_table_size;

			if (ccdc->state != ISP_PIPELINE_STREAM_STOPPED)
				return -EBUSY;

			if (copy_from_user(&fpc_t, ccdc_struct->fpc,
					   sizeof(fpc_t))) {
				ret = -EFAULT;
				goto out;
			}

			/*
			 * fpc_table_m must be 64-bytes aligned, but it's
			 * already done by iommu_vmalloc().
			 */
			fpc_table_size = fpc_t.fpnum * 4;
			fpc_table_m = iommu_vmalloc(isp->iommu, 0,
						    fpc_table_size, IOMMU_FLAG);
			if (IS_ERR_VALUE(fpc_table_m)) {
				ret = -ENOMEM;
				goto out;
			}
			fpc_table = da_to_va(isp->iommu, fpc_table_m);
			if (copy_from_user(fpc_table,
					   (__force void __user *)
					   fpc_t.fpcaddr,
					   fpc_table_size)) {
				iommu_vfree(isp->iommu, fpc_table_m);
				ret = -EFAULT;
				goto out;
			}
			fpc_t.fpcaddr = fpc_table_m;

			spin_lock_irqsave(&ccdc->lock, flags);
			fpc_table_old = ccdc->fpc_table_add_m;
			ccdc->fpc_table_add = fpc_table;
			ccdc->fpc_table_add_m = fpc_table_m;
			ispccdc_config_fpc(ccdc, fpc_t);
			spin_unlock_irqrestore(&ccdc->lock, flags);

			if (fpc_table_old != 0)
				iommu_vfree(isp->iommu, fpc_table_old);
		}
		ispccdc_enable_fpc(ccdc, 1);
	} else if (ISP_ABS_CCDC_FPC & ccdc_struct->update)
		ispccdc_enable_fpc(ccdc, 0);

	if (ISP_ABS_CCDC_CULL & ccdc_struct->update) {
		if (copy_from_user(&cull_t, ccdc_struct->cull,
				   sizeof(cull_t))) {
			ret = -EFAULT;
			goto out;
		}
		ispccdc_config_culling(ccdc, cull_t);
	}

	ret = ispccdc_config_lsc(ccdc, ccdc_struct);
	if (ret)
		goto out;

	if (ISP_ABS_CCDC_COLPTN & ccdc_struct->update)
		ispccdc_config_imgattr(ccdc, ccdc_struct->colptn);

out:
	if (ret == -EFAULT)
		dev_err(to_device(ccdc),
			"ccdc: user provided bad configuration data address");

	if (ret == -ENOMEM)
		dev_err(to_device(ccdc),
			"ccdc: can not allocate memory");

	ccdc->shadow_update = 0;
	return ret;
}

/* -----------------------------------------------------------------------------
 * Format- and pipeline-related configuration helpers
 */

/*
 * ispccdc_config_crop - Configures crop parameters for the ISP CCDC.
 * @left: Left offset of the crop area.
 * @top: Top offset of the crop area.
 * @height: Height of the crop area.
 * @width: Width of the crop area.
 *
 * The following restrictions are applied for the crop settings. If incoming
 * values do not follow these restrictions then we map the settings to the
 * closest acceptable crop value.
 * 1) Left offset is always odd. This can be avoided if we enable byte swap
 *    option for incoming data into CCDC.
 * 2) Top offset is always even.
 * 3) Crop height is always even.
 * 4) Crop width is always a multiple of 16 pixels
 */
static void ispccdc_config_crop(struct isp_ccdc_device *ccdc,
				u32 left, u32 top, u32 height, u32 width)
{
	struct isp_device *isp = to_isp_device(ccdc);
	ccdc->ccdcin_woffset = left + (left % 2);
	ccdc->ccdcin_hoffset = top + (top % 2);

	ccdc->crop_w = width - (width % 16);
	ccdc->crop_h = height + (height % 2);

	dev_dbg(isp->dev, "\n\tCCDC: Offsets L %d T %d W %d H %d\n",
		ccdc->ccdcin_woffset, ccdc->ccdcin_hoffset, ccdc->crop_w,
		ccdc->crop_h);
}

/*
 * ispccdc_config_vp - Configure the Video Port.
 * @ccdc: Pointer to ISP CCDC device.
 * @vpcfg: Structure containing the Video Port input frequency, and the 10 bit
 *         format.
 */
static void ispccdc_config_vp(struct isp_ccdc_device *ccdc,
			      struct ispccdc_vp *vpcfg)
{
	struct isp_device *isp = to_isp_device(ccdc);
	u32 fmtcfg_vp = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC,
				      ISPCCDC_FMTCFG);

	fmtcfg_vp &= ISPCCDC_FMTCFG_VPIN_MASK & ISPCCDC_FMTCFG_VPIF_FRQ_MASK;

	switch (vpcfg->bitshift_sel) {
	case BIT9_0:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_9_0;
		break;
	case BIT10_1:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_10_1;
		break;
	case BIT11_2:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_11_2;
		break;
	case BIT12_3:
		fmtcfg_vp |= ISPCCDC_FMTCFG_VPIN_12_3;
		break;
	};

	if (vpcfg->pixelclk) {
		unsigned long l3_ick = clk_get_rate(isp->clock[ISP_CLK_L3_ICK]);
		unsigned int div = l3_ick / vpcfg->pixelclk;

		div = clamp_t(unsigned int, div, 2, 6);
		fmtcfg_vp |= (div - 2) << ISPCCDC_FMTCFG_VPIF_FRQ_SHIFT;
	}

	isp_reg_writel(isp, fmtcfg_vp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMTCFG);
}

/*
 * ispccdc_enable_vp - Enable Video Port.
 * @ccdc: Pointer to ISP CCDC device.
 * @enable: 0 Disables VP, 1 Enables VP
 *
 * This is needed for outputting image to Preview, H3A and HIST ISP submodules.
 */
static void ispccdc_enable_vp(struct isp_ccdc_device *ccdc, u8 enable)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMTCFG,
		       ~ISPCCDC_FMTCFG_VPEN,
		       enable ? ISPCCDC_FMTCFG_VPEN : 0);
}

/*
 * ispccdc_config_outlineoffset - Configure memory saving output line offset
 * @ccdc: Pointer to ISP CCDC device.
 * @offset: Address offset to start a new line. Must be twice the
 *          Output width and aligned on 32 byte boundary
 * @oddeven: Specifies the odd/even line pattern to be chosen to store the
 *           output.
 * @numlines: Set the value 0-3 for +1-4lines, 4-7 for -1-4lines.
 *
 * - Configures the output line offset when stored in memory
 * - Sets the odd/even line pattern to store the output
 *    (EVENEVEN (1), ODDEVEN (2), EVENODD (3), ODDODD (4))
 * - Configures the number of even and odd line fields in case of rearranging
 * the lines.
 *
 * Returns 0 if successful, or -EINVAL if the offset is not in 32 byte
 * boundary.
 */
static int ispccdc_config_outlineoffset(struct isp_ccdc_device *ccdc,
					u32 offset, u8 oddeven, u8 numlines)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_writel(isp, offset & 0xffff,
		       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HSIZE_OFF);

	isp_reg_and(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST,
		    ~ISPCCDC_SDOFST_FINV);

	isp_reg_and(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST,
		    ~ISPCCDC_SDOFST_FOFST_4L);

	switch (oddeven) {
	case EVENEVEN:
		isp_reg_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST,
			   (numlines & 0x7) << ISPCCDC_SDOFST_LOFST0_SHIFT);
		break;
	case ODDEVEN:
		isp_reg_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST,
			   (numlines & 0x7) << ISPCCDC_SDOFST_LOFST1_SHIFT);
		break;
	case EVENODD:
		isp_reg_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST,
			   (numlines & 0x7) << ISPCCDC_SDOFST_LOFST2_SHIFT);
		break;
	case ODDODD:
		isp_reg_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDOFST,
			   (numlines & 0x7) << ISPCCDC_SDOFST_LOFST3_SHIFT);
		break;
	default:
		break;
	}
	return 0;
}

/*
 * ispccdc_set_outaddr - Set memory address to save output image
 * @ccdc: Pointer to ISP CCDC device.
 * @addr: ISP MMU Mapped 32-bit memory address aligned on 32 byte boundary.
 *
 * Sets the memory address where the output will be saved.
 */
static void ispccdc_set_outaddr(struct isp_ccdc_device *ccdc, u32 addr)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_writel(isp, addr, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SDR_ADDR);
}

/*
 * ispccdc_config_sync_if - Set CCDC sync interface params between sensor and CCDC.
 * @ccdc: Pointer to ISP CCDC device.
 * @syncif: Structure containing the sync parameters like field state, CCDC in
 *          master/slave mode, raw/yuv data, polarity of data, field, hs, vs
 *          signals.
 */
static void ispccdc_config_sync_if(struct isp_ccdc_device *ccdc,
				   struct ispccdc_syncif syncif)
{
	struct isp_device *isp = to_isp_device(ccdc);
	u32 syn_mode = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC,
				     ISPCCDC_SYN_MODE);

	syn_mode |= ISPCCDC_SYN_MODE_VDHDEN;

	if (syncif.fldstat)
		syn_mode |= ISPCCDC_SYN_MODE_FLDSTAT;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDSTAT;

	syn_mode &= ISPCCDC_SYN_MODE_DATSIZ_MASK;
	switch (syncif.datsz) {
	case DAT8:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_8;
		break;
	case DAT10:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_10;
		break;
	case DAT11:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_11;
		break;
	case DAT12:
		syn_mode |= ISPCCDC_SYN_MODE_DATSIZ_12;
		break;
	};

	if (syncif.fldmode)
		syn_mode |= ISPCCDC_SYN_MODE_FLDMODE;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDMODE;

	if (syncif.datapol)
		syn_mode |= ISPCCDC_SYN_MODE_DATAPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_DATAPOL;

	if (syncif.fldpol)
		syn_mode |= ISPCCDC_SYN_MODE_FLDPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_FLDPOL;

	if (syncif.hdpol)
		syn_mode |= ISPCCDC_SYN_MODE_HDPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_HDPOL;

	if (syncif.vdpol)
		syn_mode |= ISPCCDC_SYN_MODE_VDPOL;
	else
		syn_mode &= ~ISPCCDC_SYN_MODE_VDPOL;

	if (syncif.ccdc_mastermode) {
		syn_mode |= ISPCCDC_SYN_MODE_FLDOUT | ISPCCDC_SYN_MODE_VDHDOUT;
		isp_reg_writel(isp,
			       syncif.hs_width << ISPCCDC_HD_VD_WID_HDW_SHIFT
			       | syncif.vs_width << ISPCCDC_HD_VD_WID_VDW_SHIFT,
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_HD_VD_WID);

		isp_reg_writel(isp,
			       syncif.ppln << ISPCCDC_PIX_LINES_PPLN_SHIFT
			       | syncif.hlprf << ISPCCDC_PIX_LINES_HLPRF_SHIFT,
			       OMAP3_ISP_IOMEM_CCDC,
			       ISPCCDC_PIX_LINES);
	} else
		syn_mode &= ~(ISPCCDC_SYN_MODE_FLDOUT |
			      ISPCCDC_SYN_MODE_VDHDOUT);

	isp_reg_writel(isp, syn_mode, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SYN_MODE);

	if (!(syncif.bt_r656_en)) {
		isp_reg_and(isp, OMAP3_ISP_IOMEM_CCDC,
			    ISPCCDC_REC656IF, ~ISPCCDC_REC656IF_R656ON);
	}
}

static void ispccdc_enable(struct isp_ccdc_device *ccdc, int enable)
{
	struct isp_device *isp = to_isp_device(ccdc);

	isp_reg_and_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_PCR,
		       ~ISPCCDC_PCR_EN, enable ? ISPCCDC_PCR_EN : 0);
}

/* -----------------------------------------------------------------------------
 * Interrupt handling
 */

/*
 * ispccdc_sbl_busy - Poll idle state of CCDC and related SBL memory write bits
 * @ccdc: Pointer to ISP CCDC device.
 *
 * Returns zero if the CCDC is idle and the image has been written to
 * memory, too.
 */
static int ispccdc_sbl_busy(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);

	return ispccdc_busy(ccdc)
		| (isp_reg_readl(isp, OMAP3_ISP_IOMEM_SBL, ISPSBL_CCDC_WR_0) &
		   ISPSBL_CCDC_WR_0_DATA_READY)
		| (isp_reg_readl(isp, OMAP3_ISP_IOMEM_SBL, ISPSBL_CCDC_WR_1) &
		   ISPSBL_CCDC_WR_0_DATA_READY)
		| (isp_reg_readl(isp, OMAP3_ISP_IOMEM_SBL, ISPSBL_CCDC_WR_2) &
		   ISPSBL_CCDC_WR_0_DATA_READY)
		| (isp_reg_readl(isp, OMAP3_ISP_IOMEM_SBL, ISPSBL_CCDC_WR_3) &
		   ISPSBL_CCDC_WR_0_DATA_READY);
}

/*
 * ispccdc_sbl_wait_idle - Wait until the CCDC and related SBL are idle
 * @ccdc: Pointer to ISP CCDC device.
 * @max_wait: Max retry count in us for wait for idle/busy transition.
 */
static int ispccdc_sbl_wait_idle(struct isp_ccdc_device *ccdc,
				 unsigned int max_wait)
{
	unsigned int wait = 0;

	if (max_wait == 0)
		max_wait = 10000; /* 10 ms */

	for (wait = 0; wait <= max_wait; wait++) {
		if (!ispccdc_sbl_busy(ccdc))
			return 0;

		rmb();
		udelay(1);
	}

	return -EBUSY;
}

void ispccdc_hs_vs_isr(struct isp_ccdc_device *isp_ccdc)
{
	struct video_device *vdev = &isp_ccdc->subdev.devnode;
	struct v4l2_event event;

	memset(&event, 0, sizeof(event));
	event.type = V4L2_EVENT_OMAP3ISP_HS_VS;

	v4l2_event_queue(vdev, &event);
}

static void ispccdc_isr_buffer(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);
	struct isp_video *video = ccdc->video;
	struct isp_buffer *buffer;
	int restart = 0;

	/* The CCDC generates VD0 interrupts even when disabled (the datasheet
	 * doesn't explicitly state if that's supposed to happen or not, so it
	 * can be considered as a hardware bug or as a feature, but we have to
	 * deal with it anyway). Disabling the CCDC when no buffer is available
	 * would thus not be enough, we need to handle the situation explicitly.
	 */
	if (list_empty(&video->dmaqueue))
		goto done;

	if (ccdc->state == ISP_PIPELINE_STREAM_STOPPED)
		goto done;

	/* We're in continuous mode, and memory writes were disabled due to a
	 * buffer underrun. Reenable them now that we have a buffer. The buffer
	 * address has been set in ccdc_video_queue.
	 */
	if (ccdc->state == ISP_PIPELINE_STREAM_CONTINUOUS && ccdc->underrun) {
		ispccdc_enable(ccdc, 1);
		ccdc->underrun = 0;
		goto done;
	}

	ispccdc_enable(ccdc, 0);
	if (ispccdc_sbl_wait_idle(ccdc, 1000)) {
		dev_info(isp->dev, "CCDC won't become idle!\n");
		goto done;
	}

	buffer = isp_video_buffer_next(video, ccdc->error);
	if (buffer != NULL) {
		ispccdc_set_outaddr(ccdc, buffer->isp_addr);
		restart = 1;
	}

	if (ccdc->state == ISP_PIPELINE_STREAM_SINGLESHOT) {
		if (isp_pipeline_ready(ccdc->video_out.pipe))
			isp_pipeline_set_stream(isp, &ccdc->video_out,
						ISP_PIPELINE_STREAM_SINGLESHOT);
	} else {
		/* If an underrun occurs, the video queue operation handler will
		 * restart the CCDC. Otherwise restart it immediately.
		 */
		if (restart)
			ispccdc_enable(ccdc, 1);
	}

done:
	ccdc->error = 0;
}

/*
 * ispccdc_isr - Configure CCDC during interframe time.
 * @ccdc: Pointer to ISP CCDC device.
 *
 * Executes LSC deferred enablement before next frame starts.
 */
int ispccdc_isr(struct isp_ccdc_device *ccdc)
{
	unsigned long flags;

	if (ccdc->output & CCDC_OUTPUT_MEMORY)
		ispccdc_isr_buffer(ccdc);

	if (ccdc->stopping) {
		ccdc->stopping = 0;
		wake_up(&ccdc->wait);
		return 0;
	}

	spin_lock_irqsave(&ccdc->lock, flags);
	if (ccdc->shadow_update)
		goto out;

	if (ccdc->lsc.update_config || ccdc->lsc.update_table) {
		ispccdc_enable_lsc(ccdc, 0);
		if (!ispccdc_busy(ccdc)) {
			if (ccdc->lsc.update_config) {
				ispccdc_setup_lsc_regs(ccdc);
				ccdc->lsc.update_config = 0;
			}

			if (ccdc->lsc.update_table) {
				ccdc->lsc.table_old = ccdc->lsc.table_inuse;
				ccdc->lsc.table_inuse = ccdc->lsc.table_new;
				ccdc->lsc.table_new = 0;
				ispccdc_program_lsc_table(ccdc);
				ccdc->lsc.update_table = 0;

				/* Free old table in process context */
				schedule_work(&ccdc->lsc.table_work);
			}
		}
		ispccdc_enable_lsc(ccdc, ccdc->lsc.request_enable);
	}

out:
	spin_unlock_irqrestore(&ccdc->lock, flags);

	return 0;
}

/* -----------------------------------------------------------------------------
 * ISP video operations
 */

static int ccdc_video_queue(struct isp_video *video, struct isp_buffer *buffer)
{
	struct isp_ccdc_device *ccdc = &video->isp->isp_ccdc;

	if (!(ccdc->output & CCDC_OUTPUT_MEMORY))
		return -ENODEV;

	ispccdc_set_outaddr(ccdc, buffer->isp_addr);

	/* We now have a buffer queued on the output, restart the pipeline in
	 * on the next CCDC interrupt if running in continuous mode.
	 */
	if (ccdc->state == ISP_PIPELINE_STREAM_CONTINUOUS)
		ccdc->underrun = 1;

	return 0;
}

static const struct isp_video_operations ccdc_video_ops = {
	.queue = ccdc_video_queue,
};

/* -----------------------------------------------------------------------------
 * V4L2 subdev operations
 */

/*
 * ccdc_get_ctrl - V4L2 control get handler
 * @sd: ISP CCDC V4L2 subdevice
 * @ctrl: V4L2 control
 *
 * Return 0 on success or a negative error code otherwise.
 */
static int ccdc_get_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return -EINVAL;
}

/*
 * ccdc_set_ctrl - V4L2 control set handler
 * @sd: ISP CCDC V4L2 subdevice
 * @ctrl: V4L2 control
 *
 * Return 0 on success or a negative error code otherwise.
 */
static int ccdc_set_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	return -EINVAL;
}

/*
 * ccdc_ioctl - CCDC module private ioctl's
 * @sd: ISP CCDC V4L2 subdevice
 * @cmd: ioctl command
 * @arg: ioctl argument
 *
 * Return 0 on success or a negative error code otherwise.
 */
static long ccdc_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	int ret;

	switch (cmd) {
	case VIDIOC_PRIVATE_ISP_CCDC_CFG:
		ret = ispccdc_config(ccdc, arg);
		break;

	default:
		return -ENOIOCTLCMD;
	}

	return ret;
}

/*
 * ccdc_set_power - Power on/off the CCDC module
 * @sd: ISP CCDC V4L2 subdevice
 * @on: power on/off
 *
 * Return 0 on success or a negative error code otherwise.
 */
static int ccdc_set_power(struct v4l2_subdev *sd, int on)
{
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(ccdc);

	if (on) {
		if (!isp_get(isp))
			return -EBUSY;

		isp_reg_or(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL,
			   ISPCTRL_CCDC_RAM_EN | ISPCTRL_CCDC_CLK_EN);
		isp_reg_or(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_CFG,
			   ISPCCDC_CFG_VDLC);
	} else {
		isp_reg_and(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL,
			    ~(ISPCTRL_CCDC_CLK_EN | ISPCTRL_CCDC_RAM_EN));
		isp_put(isp);
	}

	return 0;
}

static int ccdc_subscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				struct v4l2_event_subscription *sub)
{
	if (sub->type != V4L2_EVENT_OMAP3ISP_HS_VS)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub);
}

static int ccdc_unsubscribe_event(struct v4l2_subdev *sd, struct v4l2_fh *fh,
				  struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}

static int __ispccdc_stop(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&ccdc->lock, flags);
	ccdc->stopping = 1;
	spin_unlock_irqrestore(&ccdc->lock, flags);

	ret = wait_event_timeout(ccdc->wait, ccdc->stopping == 0,
				 msecs_to_jiffies(2000));
	if (ret == 0) {
		dev_warn(isp->dev, "CCDC stop timeout!\n");
		ret = -ETIMEDOUT;
	}

	return ret;
}

/*
 * ccdc_set_stream - Enable/Disable streaming on the CCDC module
 * @sd: ISP CCDC V4L2 subdevice
 * @enable: Enable/disable stream
 *
 * When writing to memory, the CCDC hardware can't be enabled without a memory
 * buffer to write to. As the s_stream operation is called in response to a
 * STREAMON call without any buffer queued yet, just update the enabled field
 * and return immediately. The CCDC will be enabled in ccdc_isr_buffer().
 *
 * When not writing to memory enable the CCDC immediately.
 */
static int ccdc_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(ccdc);

	ccdc->underrun = 0;

	if (enable) {
		/* TODO: Don't configure the video port if all of its output
		 * links are inactive.
		 */
		ispccdc_config_vp(ccdc, &ccdc->vpcfg);
		ispccdc_enable_vp(ccdc, 1);
	}

	ispccdc_enable_lsc(ccdc, enable && ccdc->lsc.request_enable);

	switch (enable) {
	case ISP_PIPELINE_STREAM_CONTINUOUS:
		if (ccdc->output & CCDC_OUTPUT_MEMORY)
			isp_sbl_enable(isp, OMAP3_ISP_SBL_CCDC_WRITE);
		else
			ispccdc_enable(ccdc, 1);
		break;

	case ISP_PIPELINE_STREAM_SINGLESHOT:
		if (ccdc->output & CCDC_OUTPUT_MEMORY &&
		    ccdc->state != ISP_PIPELINE_STREAM_SINGLESHOT)
			isp_sbl_enable(isp, OMAP3_ISP_SBL_CCDC_WRITE);

		ispccdc_enable(ccdc, 1);
		break;

	case ISP_PIPELINE_STREAM_STOPPED:
		ispccdc_enable(ccdc, 0);
		if (ccdc->output & CCDC_OUTPUT_MEMORY) {
			__ispccdc_stop(ccdc);
			isp_sbl_disable(isp, OMAP3_ISP_SBL_CCDC_WRITE);
		}
		break;
	}

	ccdc->state = enable;
	return 0;
}

/* CCDC formats descriptions */
static const u32 ccdc_sgrbg_pattern =
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP0PLC0_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP0PLC1_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP0PLC2_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP0PLC3_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP1PLC0_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP1PLC1_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP1PLC2_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP1PLC3_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP2PLC0_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP2PLC1_SHIFT |
	ISPCCDC_COLPTN_Gr_Cy << ISPCCDC_COLPTN_CP2PLC2_SHIFT |
	ISPCCDC_COLPTN_R_Ye  << ISPCCDC_COLPTN_CP2PLC3_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP3PLC0_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP3PLC1_SHIFT |
	ISPCCDC_COLPTN_B_Mg  << ISPCCDC_COLPTN_CP3PLC2_SHIFT |
	ISPCCDC_COLPTN_Gb_G  << ISPCCDC_COLPTN_CP3PLC3_SHIFT;

/*
 * ccdc_enum_mbus_code - Handle pixel format enumeration
 * @sd     : pointer to v4l2 subdev structure
 * @code   : pointer to v4l2_subdev_pad_mbus_code_enum structure
 * return -EINVAL or zero on success
 */
static int ccdc_enum_mbus_code(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_mbus_code_enum *code)
{
	if (code->pad >= CCDC_PADS_NUM || code->index > 0)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}

static struct v4l2_mbus_framefmt *
__ccdc_get_format(struct isp_ccdc_device *ccdc, unsigned int pad,
		  enum v4l2_subdev_format which)
{
	if (which != V4L2_SUBDEV_FORMAT_PROBE &&
	    which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return NULL;

	if (pad >= CCDC_PADS_NUM)
		return NULL;

	return &ccdc->formats[pad][which];
}

/*
 * ccdc_get_format - Retrieve the video format on a pad
 * @sd : ISP CCDC V4L2 subdevice
 * @pad: Pad number
 * @fmt: Format
 *
 * Return 0 on success or -EINVAL if the pad is invalid or doesn't correspond
 * to the format type.
 */
static int ccdc_get_format(struct v4l2_subdev *sd, unsigned int pad,
			   struct v4l2_mbus_framefmt *fmt,
			   enum v4l2_subdev_format which)
{
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format = __ccdc_get_format(ccdc, pad, which);

	if (format == NULL)
		return -EINVAL;

	memcpy(fmt, format, sizeof(*fmt));
	return 0;
}

/*
 * ccdc_try_format - Try video format on a pad
 * @sd : ISP CCDC V4L2 subdevice
 * @pad: Pad number
 * @fmt: Format
 */
static void
ccdc_try_format(struct v4l2_subdev *sd, unsigned int pad,
		struct v4l2_mbus_framefmt *fmt, enum v4l2_subdev_format which)
{
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	unsigned int width = fmt->width;
	unsigned int height = fmt->height;

	switch (pad) {
	case CCDC_PAD_SINK:
		/* Check if the requested pixel format is supported.
		 * TODO: If the CCDC output formatter pad is connected directly
		 * to the resizer, only YUV formats can be used.
		 */
		fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;

		/* Clamp the input size. */
		fmt->width = clamp_t(u32, width, 32, 4096);
		fmt->height = clamp_t(u32, height, 32, 4096);
		break;

	case CCDC_PAD_SOURCE_OF:
		format = __ccdc_get_format(ccdc, CCDC_PAD_SINK, which);
		memcpy(fmt, format, sizeof(*fmt));

		/* The data formatter truncates the number of horizontal output
		 * pixels to a multiple of 16. To avoid clipping data, allow
		 * callers to request an output size bigger than the input size
		 * up to the nearest multiple of 16.
		 */
		fmt->width = clamp_t(u32, width, 32, (fmt->width + 15) & ~15);
		fmt->width &= ~15;
		fmt->height = clamp_t(u32, height, 32, fmt->height);
		break;

	case CCDC_PAD_SOURCE_VP:
		format = __ccdc_get_format(ccdc, CCDC_PAD_SINK, which);
		memcpy(fmt, format, sizeof(*fmt));

		/* The number of lines that can be clocked out from the video
		 * port output must be at least one line less than the number
		 * of input lines.
		 */
		fmt->width = clamp_t(u32, width, 32, fmt->width);
		fmt->height = clamp_t(u32, height, 32, fmt->height - 1);
		break;
	}

	/* Data is written to memory unpacked, each 10-bit pixel is stored on
	 * 2 bytes.
	 */
	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->field = V4L2_FIELD_NONE;
}

/*
 * ccdc_set_format - Set the video format on a pad
 * @sd : ISP CCDC V4L2 subdevice
 * @pad: Pad number
 * @fmt: Format
 *
 * Return 0 on success or -EINVAL if the pad is invalid or doesn't correspond
 * to the format type.
 */
static int ccdc_set_format(struct v4l2_subdev *sd, unsigned int pad,
			   struct v4l2_mbus_framefmt *fmt,
			   enum v4l2_subdev_format which)
{
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(ccdc);
	struct v4l2_mbus_framefmt *format = __ccdc_get_format(ccdc, pad, which);
	unsigned long flags;

	if (format == NULL)
		return -EINVAL;

	ccdc_try_format(sd, pad, fmt, which);
	memcpy(format, fmt, sizeof(*format));

	if (which == V4L2_SUBDEV_FORMAT_PROBE)
		return 0;

	switch (pad) {
	case CCDC_PAD_SINK:
		/* Mosaic filter. Hardcode GRBG Bayer pattern for now. */
		ispccdc_config_imgattr(ccdc, ccdc_sgrbg_pattern);

		/* Generate VD0 on the last line of the image and VD1 on the
		 * first line.
		 */
		isp_reg_writel(isp, ((fmt->height - 2)
				<< ISPCCDC_VDINT_0_SHIFT) |
			       (0 << ISPCCDC_VDINT_1_SHIFT),
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VDINT);

		ispccdc_print_status(ccdc);
		break;

	case CCDC_PAD_SOURCE_OF:
		isp_reg_writel(isp, (0 << ISPCCDC_HORZ_INFO_SPH_SHIFT) |
			       ((fmt->width - 1)
				<< ISPCCDC_HORZ_INFO_NPH_SHIFT),
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_HORZ_INFO);
		isp_reg_writel(isp, 0 << ISPCCDC_VERT_START_SLV0_SHIFT,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VERT_START);
		isp_reg_writel(isp, (fmt->height - 1)
				<< ISPCCDC_VERT_LINES_NLV_SHIFT,
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VERT_LINES);

		ispccdc_config_outlineoffset(ccdc,
				fmt->width * ISP_BYTES_PER_PIXEL, 0, 0);
		break;

	case CCDC_PAD_SOURCE_VP:
		isp_reg_writel(isp, (0 << ISPCCDC_FMT_HORZ_FMTSPH_SHIFT) |
			       (fmt->width << ISPCCDC_FMT_HORZ_FMTLNH_SHIFT),
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_HORZ);
		isp_reg_writel(isp, (0 << ISPCCDC_FMT_VERT_FMTSLV_SHIFT) |
			       ((fmt->height + 1)
				<< ISPCCDC_FMT_VERT_FMTLNV_SHIFT),
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMT_VERT);

		isp_reg_writel(isp, (fmt->width
				<< ISPCCDC_VP_OUT_HORZ_NUM_SHIFT) |
			       (fmt->height << ISPCCDC_VP_OUT_VERT_NUM_SHIFT),
			       OMAP3_ISP_IOMEM_CCDC, ISPCCDC_VP_OUT);

		/* Setup LSC. Disable it if not supported for the selected
		 * resolution.
		 */
		spin_lock_irqsave(&ccdc->lock, flags);
		if (ccdc->lsc.request_enable &&
		    ispccdc_validate_config_lsc(ccdc, &ccdc->lsc.config))
			ccdc->lsc.request_enable = 0;
		spin_unlock_irqrestore(&ccdc->lock, flags);

		ispccdc_setup_lsc(ccdc);
		break;
	}

	return 0;
}

/* V4L2 subdev core operations */
static const struct v4l2_subdev_core_ops ccdc_v4l2_core_ops = {
	.g_ctrl = ccdc_get_ctrl,
	.s_ctrl = ccdc_set_ctrl,
	.ioctl = ccdc_ioctl,
	.s_power = ccdc_set_power,
	.subscribe_event = ccdc_subscribe_event,
	.unsubscribe_event = ccdc_unsubscribe_event,
};

/* V4L2 subdev video operations */
static const struct v4l2_subdev_video_ops ccdc_v4l2_video_ops = {
	.s_stream = ccdc_set_stream,
};

/* V4L2 subdev pad operations */
static const struct v4l2_subdev_pad_ops ccdc_v4l2_pad_ops = {
	.enum_mbus_code = ccdc_enum_mbus_code,
	.get_fmt = ccdc_get_format,
	.set_fmt = ccdc_set_format,
};

/* V4L2 subdev operations */
static const struct v4l2_subdev_ops ccdc_v4l2_ops = {
	.core = &ccdc_v4l2_core_ops,
	.video = &ccdc_v4l2_video_ops,
	.pad = &ccdc_v4l2_pad_ops,
};

/* -----------------------------------------------------------------------------
 * Media entity operations
 */

/*
 * ccdc_link_setup - Setup CCDC connections
 * @entity: CCDC media entity
 * @local: Pad at the local end of the link
 * @remote: Pad at the remote end of the link
 * @flags: Link flags
 *
 * return -EINVAL or zero on success
 */
static int ccdc_link_setup(struct media_entity *entity,
			   const struct media_entity_pad *local,
			   const struct media_entity_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct isp_ccdc_device *ccdc = v4l2_get_subdevdata(sd);
	struct isp_device *isp = to_isp_device(ccdc);
	enum ccdc_input_entity input;
	u32 syn_mode;

	syn_mode = isp_reg_readl(isp, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SYN_MODE);

	/* Use the raw, unprocessed data when writing to memory. The H3A and
	 * histogram modules are still fed with lens shading corrected data.
	 */
	syn_mode &= ~ISPCCDC_SYN_MODE_VP2SDR;

	switch (local->index | (remote->entity->type << 16)) {
	case CCDC_PAD_SINK | (MEDIA_ENTITY_TYPE_SUBDEV << 16):
		/* Read from the sensor (parallel interface), CCP2, CSI2a or
		 * CSI2c.
		 */
		if (!(flags & MEDIA_LINK_FLAG_ACTIVE)) {
			ccdc->input = CCDC_INPUT_NONE;
			break;
		}

		if (ccdc->input != CCDC_INPUT_NONE)
			return -EBUSY;

		if (remote->entity == &isp->isp_ccp2.subdev.entity)
			input = CCDC_INPUT_CCP2B;
		else if (remote->entity == &isp->isp_csi2a.subdev.entity)
			input = CCDC_INPUT_CSI2A;
		else if (remote->entity == &isp->isp_csi2c.subdev.entity)
			input = CCDC_INPUT_CSI2C;
		else
			input = CCDC_INPUT_PARALLEL;

		isp_select_bridge_input(isp, input);
		ccdc->input = input;
		break;

	case CCDC_PAD_SOURCE_VP | (MEDIA_ENTITY_TYPE_SUBDEV << 16):
		/* Write to preview engine, histogram and H3A. When none of
		 * those links are active, the video port can be disabled.
		 */
		if (flags & MEDIA_LINK_FLAG_ACTIVE)
			ccdc->output |= CCDC_OUTPUT_PREVIEW;
		else
			ccdc->output &= ~CCDC_OUTPUT_PREVIEW;
		break;

	case CCDC_PAD_SOURCE_OF | (MEDIA_ENTITY_TYPE_NODE << 16):
		/* Write to memory */
		if (flags & MEDIA_LINK_FLAG_ACTIVE) {
			ccdc->output |= CCDC_OUTPUT_MEMORY;
			syn_mode |= ISPCCDC_SYN_MODE_WEN;
			ccdc->video = container_of(remote->entity,
					struct isp_video, video.entity);
		} else {
			ccdc->output &= ~CCDC_OUTPUT_MEMORY;
			syn_mode &= ~ISPCCDC_SYN_MODE_WEN;
		}
		break;

	case CCDC_PAD_SOURCE_OF | (MEDIA_ENTITY_TYPE_SUBDEV << 16):
		/* Write to resizer */
		if (flags & MEDIA_LINK_FLAG_ACTIVE) {
			ccdc->output |= CCDC_OUTPUT_RESIZER;
			syn_mode |= ISPCCDC_SYN_MODE_SDR2RSZ;
		} else {
			ccdc->output &= ~CCDC_OUTPUT_RESIZER;
			syn_mode &= ~ISPCCDC_SYN_MODE_SDR2RSZ;
		}
		break;

	default:
		return -EINVAL;
	}

	isp_reg_writel(isp, syn_mode, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_SYN_MODE);
	return 0;
}

/* media operations */
static const struct media_entity_operations ccdc_media_ops = {
	.link_setup = ccdc_link_setup,
	.set_power = v4l2_subdev_set_power,
};

/*
 * isp_ccdc_init_entities - Initialize V4L2 subdev and media entity
 * @ccdc: ISP CCDC module
 *
 * Return 0 on success and a negative error code on failure.
 */
static int isp_ccdc_init_entities(struct isp_ccdc_device *ccdc)
{
	struct v4l2_subdev *sd = &ccdc->subdev;
	struct media_entity_pad *pads = ccdc->pads;
	struct media_entity *me = &sd->entity;
	int ret;

	ccdc->input = CCDC_INPUT_NONE;

	v4l2_subdev_init(sd, &ccdc_v4l2_ops);
	strlcpy(sd->name, "OMAP3 ISP CCDC", sizeof(sd->name));
	sd->grp_id = 1 << 16;	/* group ID for isp subdevs */
	v4l2_set_subdevdata(sd, ccdc);
	sd->flags |= V4L2_SUBDEV_USES_EVENTS;
	sd->nevents = OMAP3ISP_CCDC_NEVENTS;

	pads[CCDC_PAD_SINK].type = MEDIA_PAD_TYPE_INPUT;
	pads[CCDC_PAD_SOURCE_VP].type = MEDIA_PAD_TYPE_OUTPUT;
	pads[CCDC_PAD_SOURCE_OF].type = MEDIA_PAD_TYPE_OUTPUT;

	me->ops = &ccdc_media_ops;
	ret = media_entity_init(me, CCDC_PADS_NUM, pads, 0);
	if (ret < 0)
		return ret;

	ccdc->video_out.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ccdc->video_out.ops = &ccdc_video_ops;
	ccdc->video_out.isp = to_isp_device(ccdc);
	ccdc->video_out.capture_mem = PAGE_ALIGN(4096 * 4096) * 3;
	ccdc->video_out.alignment = 32;

	ret = isp_video_init(&ccdc->video_out, "CCDC");
	if (ret < 0)
		return ret;

	/* Connect the CCDC subdev to the video node. */
	ret = media_entity_create_link(&ccdc->subdev.entity, CCDC_PAD_SOURCE_OF,
			&ccdc->video_out.video.entity, 0, 0);
	if (ret < 0)
		return ret;

	return 0;
}

void isp_ccdc_unregister_entities(struct isp_ccdc_device *ccdc)
{
	media_entity_cleanup(&ccdc->subdev.entity);

	v4l2_device_unregister_subdev(&ccdc->subdev);
	isp_video_unregister(&ccdc->video_out);
}

int isp_ccdc_register_entities(struct isp_ccdc_device *ccdc,
	struct v4l2_device *vdev)
{
	int ret;

	/* Register the subdev and video node. */
	ret = v4l2_device_register_subdev(vdev, &ccdc->subdev);
	if (ret < 0)
		goto error;

	ret = isp_video_register(&ccdc->video_out, vdev);
	if (ret < 0)
		goto error;

	return 0;

error:
	isp_ccdc_unregister_entities(ccdc);
	return ret;
}

/* -----------------------------------------------------------------------------
 * ISP CCDC initialisation and cleanup
 */

/*
 * isp_ccdc_init_lsc - CCDC LSC initialization.
 * @ccdc: ISP CCDC module
 *
 * Return 0 on success or -ENOMEM is memory for the LSC table can't be
 * allocated.
 */
static int isp_ccdc_init_lsc(struct isp_ccdc_device *ccdc)
{
	struct isp_device *isp = to_isp_device(ccdc);
	void *p;

	ccdc->lsc.update_config = 0;
	ccdc->lsc.request_enable = 1;

	ccdc->lsc.config.initial_x = 0;
	ccdc->lsc.config.initial_y = 0;
	ccdc->lsc.config.gain_mode_n = 0x6;
	ccdc->lsc.config.gain_mode_m = 0x6;
	ccdc->lsc.config.gain_format = 0x4;
	ccdc->lsc.config.offset = 0x60;
	ccdc->lsc.config.size = LSC_TABLE_INIT_SIZE;

	ccdc->lsc.update_table = 0;
	ccdc->lsc.table_new = 0;
	ccdc->lsc.table_inuse = iommu_vmalloc(isp->iommu, 0,
					LSC_TABLE_INIT_SIZE, IOMMU_FLAG);
	if (IS_ERR_VALUE(ccdc->lsc.table_inuse)) {
		ccdc->lsc.table_inuse = 0;
		return -ENOMEM;
	}
	p = da_to_va(isp->iommu, ccdc->lsc.table_inuse);
	memset(p, 0x40, LSC_TABLE_INIT_SIZE);

	return 0;
}

/**
 * isp_ccdc_init - CCDC module initialization.
 * @dev: Device pointer specific to the OMAP3 ISP.
 *
 * TODO: Get the initialisation values from platform data.
 *
 * Return 0 on success or a negative error code otherwise.
 **/
int isp_ccdc_init(struct isp_device *isp)
{
	struct isp_ccdc_device *ccdc = &isp->isp_ccdc;
	int ret;

	spin_lock_init(&ccdc->lock);
	init_waitqueue_head(&ccdc->wait);

	ccdc->shadow_update = 0;
	ispccdc_config_crop(ccdc, 0, 0, 0, 0);

	INIT_WORK(&ccdc->lsc.table_work, ispccdc_free_lsc_table_work);
	ret = isp_ccdc_init_lsc(ccdc);
	if (ret < 0)
		return ret;

	ccdc->fpc_table_add_m = 0;

	ccdc->syncif.ccdc_mastermode = 0;
	ccdc->syncif.datapol = 0;
	ccdc->syncif.datsz = DAT10;
	ccdc->syncif.fldmode = 0;
	ccdc->syncif.fldout = 0;
	ccdc->syncif.fldpol = 0;
	ccdc->syncif.fldstat = 0;
	ccdc->syncif.hdpol = 0;
	ccdc->syncif.vdpol = 0;

	ccdc->blkcfg.oblen = 0;
	ccdc->blkcfg.dcsubval = 64;

	ccdc->vpcfg.bitshift_sel = BIT9_0;
	ccdc->vpcfg.pixelclk = 0;

	ispccdc_config_sync_if(ccdc, ccdc->syncif);
	ispccdc_config_black_clamp(ccdc, ccdc->blkcfg);

	ret = isp_ccdc_init_entities(ccdc);
	if (ret < 0) {
		iommu_vfree(isp->iommu, ccdc->lsc.table_inuse);
		return ret;
	}

	return 0;
}

/**
 * isp_ccdc_cleanup - CCDC module cleanup.
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
void isp_ccdc_cleanup(struct isp_device *isp)
{
	struct isp_ccdc_device *ccdc = &isp->isp_ccdc;

	if (ccdc->lsc.table_inuse != 0)
		iommu_vfree(isp->iommu, ccdc->lsc.table_inuse);
	if (ccdc->lsc.table_new != 0)
		iommu_vfree(isp->iommu, ccdc->lsc.table_new);
	/* Free lsc old table */
	flush_work(&ccdc->lsc.table_work);

	if (ccdc->fpc_table_add_m != 0)
		iommu_vfree(isp->iommu, ccdc->fpc_table_add_m);
}
