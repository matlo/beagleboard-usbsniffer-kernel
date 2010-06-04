/*
 * isp.c
 *
 * Driver Library for ISP Control module in TI's OMAP3 Camera ISP
 * ISP interface and IRQ related APIs are defined here.
 *
 * Copyright (C) 2009 Texas Instruments.
 * Copyright (C) 2009 Nokia.
 *
 * Contributors:
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

#include <asm/cacheflush.h>

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>
#include <linux/device.h>
#include <linux/slab.h>

#include "isp.h"
#include "ispreg.h"
#include "ispccdc.h"
#include "isppreview.h"
#include "ispresizer.h"
#include "ispcsi2.h"
#include "ispccp2.h"
#include "isph3a.h"
#include "isphist.h"

static void isp_save_ctx(struct isp_device *isp);

static void isp_restore_ctx(struct isp_device *isp);

const static struct isp_res_mapping isp_res_maps[] = {
	{
		.isp_rev = ISP_REVISION_2_0,
		.map = 1 << OMAP3_ISP_IOMEM_MAIN |
		       1 << OMAP3_ISP_IOMEM_CBUFF |
		       1 << OMAP3_ISP_IOMEM_CCP2 |
		       1 << OMAP3_ISP_IOMEM_CCDC |
		       1 << OMAP3_ISP_IOMEM_HIST |
		       1 << OMAP3_ISP_IOMEM_H3A |
		       1 << OMAP3_ISP_IOMEM_PREV |
		       1 << OMAP3_ISP_IOMEM_RESZ |
		       1 << OMAP3_ISP_IOMEM_SBL |
		       1 << OMAP3_ISP_IOMEM_CSI2A_REGS1 |
		       1 << OMAP3_ISP_IOMEM_CSIPHY2,
	},
	{
		.isp_rev = ISP_REVISION_15_0,
		.map = 1 << OMAP3_ISP_IOMEM_MAIN |
		       1 << OMAP3_ISP_IOMEM_CBUFF |
		       1 << OMAP3_ISP_IOMEM_CCP2 |
		       1 << OMAP3_ISP_IOMEM_CCDC |
		       1 << OMAP3_ISP_IOMEM_HIST |
		       1 << OMAP3_ISP_IOMEM_H3A |
		       1 << OMAP3_ISP_IOMEM_PREV |
		       1 << OMAP3_ISP_IOMEM_RESZ |
		       1 << OMAP3_ISP_IOMEM_SBL |
		       1 << OMAP3_ISP_IOMEM_CSI2A_REGS1 |
		       1 << OMAP3_ISP_IOMEM_CSIPHY2 |
		       1 << OMAP3_ISP_IOMEM_CSI2A_REGS2 |
		       1 << OMAP3_ISP_IOMEM_CSI2C_REGS1 |
		       1 << OMAP3_ISP_IOMEM_CSIPHY1 |
		       1 << OMAP3_ISP_IOMEM_CSI2C_REGS2,
	},
};

/* Structure for saving/restoring ISP module registers */
static struct isp_reg isp_reg_list[] = {
	{OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_GRESET_LENGTH, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_PSTRB_REPLAY, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_CTRL, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_CTRL, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_FRAME, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_PSTRB_DELAY, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_STRB_DELAY, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_SHUT_DELAY, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_PSTRB_LENGTH, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_STRB_LENGTH, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_SHUT_LENGTH, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF_SYSCONFIG, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF_IRQENABLE, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF0_CTRL, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF1_CTRL, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF0_START, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF1_START, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF0_END, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF1_END, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF0_WINDOWSIZE, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF1_WINDOWSIZE, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF0_THRESHOLD, 0},
	{OMAP3_ISP_IOMEM_CBUFF, ISP_CBUFF1_THRESHOLD, 0},
	{0, ISP_TOK_TERM, 0}
};

/**
 * isp_flush - Post pending L3 bus writes by doing a register readback
 * @dev: Device pointer specific to the OMAP3 ISP.
 *
 * In order to force posting of pending writes, we need to write and
 * readback the same register, in this case the revision register.
 *
 * See this link for reference:
 *   http://www.mail-archive.com/linux-omap@vger.kernel.org/msg08149.html
 **/
void isp_flush(struct isp_device *isp)
{
	isp_reg_writel(isp, 0, OMAP3_ISP_IOMEM_MAIN, ISP_REVISION);
	isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_REVISION);
}

/*
 * isp_enable_interrupts - Enable ISP interrupts.
 * @dev: Device pointer specific to the OMAP3 ISP.
 */
static void isp_enable_interrupts(struct isp_device *isp)
{
	static const u32 irq = IRQ0ENABLE_CSIA_IRQ
			     | IRQ0ENABLE_CSIB_IRQ
			     | IRQ0ENABLE_CCDC_LSC_PREF_ERR_IRQ
			     | IRQ0ENABLE_CCDC_VD0_IRQ
			     | IRQ0ENABLE_HIST_DONE_IRQ
			     | IRQ0ENABLE_H3A_AWB_DONE_IRQ
			     | IRQ0ENABLE_H3A_AF_DONE_IRQ
			     | IRQ0ENABLE_PRV_DONE_IRQ
			     | IRQ0ENABLE_RSZ_DONE_IRQ;

	isp_reg_writel(isp, irq, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
	isp_reg_writel(isp, irq, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE);
}

/*
 * isp_disable_interrupts - Disable ISP interrupts.
 * @dev: Device pointer specific to the OMAP3 ISP.
 */
static void isp_disable_interrupts(struct isp_device *isp)
{
	isp_reg_writel(isp, 0, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE);
}

/**
 * isp_set_xclk - Configures the specified cam_xclk to the desired frequency.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @xclk: Desired frequency of the clock in Hz. 0 = stable low, 1 is stable high
 * @xclksel: XCLK to configure (0 = A, 1 = B).
 *
 * Configures the specified MCLK divisor in the ISP timing control register
 * (TCTRL_CTRL) to generate the desired xclk clock value.
 *
 * Divisor = cam_mclk_hz / xclk
 *
 * Returns the final frequency that is actually being generated
 **/
u32 isp_set_xclk(struct isp_device *isp, u32 xclk, u8 xclksel)
{
	u32 divisor;
	u32 currentxclk;
	unsigned long mclk_hz;

	if (!isp_get(isp))
		return 0;

	mclk_hz = clk_get_rate(isp->clock[ISP_CLK_CAM_MCLK]);

	if (xclk >= mclk_hz) {
		divisor = ISPTCTRL_CTRL_DIV_BYPASS;
		currentxclk = mclk_hz;
	} else if (xclk >= 2) {
		divisor = mclk_hz / xclk;
		if (divisor >= ISPTCTRL_CTRL_DIV_BYPASS)
			divisor = ISPTCTRL_CTRL_DIV_BYPASS - 1;
		currentxclk = mclk_hz / divisor;
	} else {
		divisor = xclk;
		currentxclk = 0;
	}

	switch (xclksel) {
	case 0:
		isp_reg_and_or(isp, OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_CTRL,
			       ~ISPTCTRL_CTRL_DIVA_MASK,
			       divisor << ISPTCTRL_CTRL_DIVA_SHIFT);
		dev_dbg(isp->dev, "isp_set_xclk(): cam_xclka set to %d Hz\n",
			currentxclk);
		break;
	case 1:
		isp_reg_and_or(isp, OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_CTRL,
			       ~ISPTCTRL_CTRL_DIVB_MASK,
			       divisor << ISPTCTRL_CTRL_DIVB_SHIFT);
		dev_dbg(isp->dev, "isp_set_xclk(): cam_xclkb set to %d Hz\n",
			currentxclk);
		break;
	default:
		isp_put(isp);
		dev_dbg(isp->dev, "ISP_ERR: isp_set_xclk(): Invalid requested "
			"xclk. Must be 0 (A) or 1 (B).\n");
		return -EINVAL;
	}

	/* Do we go from stable whatever to clock? */
	if (divisor >= 2 && isp->xclk_divisor[xclksel] < 2)
		isp_get(isp);
	/* Stopping the clock. */
	else if (divisor < 2 && isp->xclk_divisor[xclksel] >= 2)
		isp_put(isp);

	isp->xclk_divisor[xclksel] = divisor;

	isp_put(isp);

	return currentxclk;
}
EXPORT_SYMBOL(isp_set_xclk);

/**
 * isp_power_settings - Sysconfig settings, for Power Management.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @idle: Consider idle state.
 *
 * Sets the power settings for the ISP, and SBL bus.
 **/
static void isp_power_settings(struct isp_device *isp, int idle)
{
	if (idle) {
		isp_reg_writel(isp,
			       (ISP_SYSCONFIG_MIDLEMODE_SMARTSTANDBY <<
				ISP_SYSCONFIG_MIDLEMODE_SHIFT),
			       OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG);
		if (cpu_rev_eq(34xx, OMAP34XX_ES_1_0)) {
			isp_reg_writel(isp, ISPCSI1_AUTOIDLE |
				       (ISPCSI1_MIDLEMODE_SMARTSTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
				       OMAP3_ISP_IOMEM_CSI2A_REGS1,
				       ISP_CSIA_SYSCONFIG);
			isp_reg_writel(isp, ISPCSI1_AUTOIDLE |
				       (ISPCSI1_MIDLEMODE_SMARTSTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
				       OMAP3_ISP_IOMEM_CCP2,
				       ISP_CSIB_SYSCONFIG);
		}
		isp_reg_writel(isp, ISPCTRL_SBL_AUTOIDLE, OMAP3_ISP_IOMEM_MAIN,
			       ISP_CTRL);

	} else {
		isp_reg_writel(isp,
			       (ISP_SYSCONFIG_MIDLEMODE_FORCESTANDBY <<
				ISP_SYSCONFIG_MIDLEMODE_SHIFT),
			       OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG);
		if (cpu_rev_eq(34xx, OMAP34XX_ES_1_0)) {
			isp_reg_writel(isp, ISPCSI1_AUTOIDLE |
				       (ISPCSI1_MIDLEMODE_FORCESTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
				       OMAP3_ISP_IOMEM_CSI2A_REGS1,
				       ISP_CSIA_SYSCONFIG);

			isp_reg_writel(isp, ISPCSI1_AUTOIDLE |
				       (ISPCSI1_MIDLEMODE_FORCESTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
				       OMAP3_ISP_IOMEM_CCP2,
				       ISP_CSIB_SYSCONFIG);
		}

		isp_reg_writel(isp, ISPCTRL_SBL_AUTOIDLE, OMAP3_ISP_IOMEM_MAIN,
			       ISP_CTRL);
	}
}

/*
 * Select the input to the bridge and lane shifter. Valid inputs are
 *
 * CCDC_INPUT_PARALLEL: Parallel interface
 * CCDC_INPUT_CSI2A: CSI2a receiver
 * CCDC_INPUT_CCP2B: CCP2b receiver
 * CCDC_INPUT_CSI2C: CSI2c receiver
 *
 * The bridge and lane shifter are configured according to the selected input
 * and the ISP platform data.
 */
void isp_select_bridge_input(struct isp_device *isp,
			     enum ccdc_input_entity input)
{
	u32 ispctrl_val = isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL);

	ispctrl_val &= ISPCTRL_SHIFT_MASK;
	ispctrl_val &= ~ISPCTRL_PAR_CLK_POL_INV;
	ispctrl_val &= ISPCTRL_PAR_SER_CLK_SEL_MASK;
	ispctrl_val &= ~ISPCTRL_PAR_BRIDGE_MASK;

	switch (input) {
	case CCDC_INPUT_PARALLEL:
		ispctrl_val |= ISPCTRL_PAR_SER_CLK_SEL_PARALLEL;
		ispctrl_val |= isp->pdata->parallel.data_lane_shift
			<< ISPCTRL_SHIFT_SHIFT;
		ispctrl_val |= isp->pdata->parallel.clk_pol
			<< ISPCTRL_PAR_CLK_POL_SHIFT;
		ispctrl_val |= isp->pdata->parallel.bridge
			<< ISPCTRL_PAR_BRIDGE_SHIFT;
		break;

	case CCDC_INPUT_CSI2A:
		ispctrl_val |= ISPCTRL_PAR_SER_CLK_SEL_CSIA;
		ispctrl_val |= isp->pdata->csi2a.data_lane_shift
			<< ISPCTRL_SHIFT_SHIFT;
		break;

	case CCDC_INPUT_CCP2B:
		ispctrl_val |= ISPCTRL_PAR_SER_CLK_SEL_CSIB;
		ispctrl_val |= isp->pdata->ccp2.data_lane_shift
			<< ISPCTRL_SHIFT_SHIFT;
		break;

	case CCDC_INPUT_CSI2C:
		ispctrl_val |= ISPCTRL_PAR_SER_CLK_SEL_CSIC;
		ispctrl_val |= isp->pdata->csi2c.data_lane_shift
			<< ISPCTRL_SHIFT_SHIFT;
		break;

	default:
		return;
	}

	ispctrl_val &= ~ISPCTRL_SYNC_DETECT_MASK;
	ispctrl_val |= ISPCTRL_SYNC_DETECT_VSRISE;

	isp_reg_writel(isp, ispctrl_val, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL);
}

/**
 * isp_set_pixel_clock - Configures the ISP pixel clock
 * @isp: OMAP3 ISP device
 * @pixelclk: Average pixel clock in Hz
 *
 * Set the average pixel clock required by the sensor. The ISP will use the
 * lowest possible memory bandwidth settings compatible with the clock.
 **/
void isp_set_pixel_clock(struct isp_device *isp, unsigned int pixelclk)
{
	isp->isp_ccdc.vpcfg.pixelclk = pixelclk;
}
EXPORT_SYMBOL(isp_set_pixel_clock);

void isphist_dma_done(struct isp_device *isp)
{
	if (ispccdc_busy(&isp->isp_ccdc) || ispstat_busy(&isp->isp_hist)) {
		/* Histogram cannot be enabled in this frame anymore */
		isp->isp_hist.buf_err = 1;
		dev_dbg(isp->dev, "hist: Out of synchronization with "
				  "CCDC. Ignoring next buffer.\n");
	}
}

static inline void isp_isr_dbg(struct isp_device *isp, u32 irqstatus)
{
	static const char *name[] = {
		"CSIA_IRQ",
		"res1",
		"res2",
		"CSIB_LCM_IRQ",
		"CSIB_IRQ",
		"res5",
		"res6",
		"res7",
		"CCDC_VD0_IRQ",
		"CCDC_VD1_IRQ",
		"CCDC_VD2_IRQ",
		"CCDC_ERR_IRQ",
		"H3A_AF_DONE_IRQ",
		"H3A_AWB_DONE_IRQ",
		"res14",
		"res15",
		"HIST_DONE_IRQ",
		"CCDC_LSC_DONE",
		"CCDC_LSC_PREFETCH_COMPLETED",
		"CCDC_LSC_PREFETCH_ERROR",
		"PRV_DONE_IRQ",
		"CBUFF_IRQ",
		"res22",
		"res23",
		"RSZ_DONE_IRQ",
		"OVF_IRQ",
		"res26",
		"res27",
		"MMU_ERR_IRQ",
		"OCP_ERR_IRQ",
		"SEC_ERR_IRQ",
		"HS_VS_IRQ",
	};
	int i;

	dev_dbg(isp->dev, "");

	for (i = 0; i < ARRAY_SIZE(name); i++) {
		if ((1 << i) & irqstatus)
			printk(KERN_CONT "%s ", name[i]);
	}
	printk(KERN_CONT "\n");
}

static void isp_isr_sbl(struct isp_device *isp)
{
	struct device *dev = isp->dev;
	u32 sbl_pcr;

	/*
	 * Handle shared buffer logic overflows for video buffers.
	 * ISPSBL_PCR_CCDCPRV_2_RSZ_OVF can be safely ignored.
	 */
	sbl_pcr = isp_reg_readl(isp, OMAP3_ISP_IOMEM_SBL, ISPSBL_PCR);
	isp_reg_writel(isp, sbl_pcr, OMAP3_ISP_IOMEM_SBL, ISPSBL_PCR);
	sbl_pcr &= ~ISPSBL_PCR_CCDCPRV_2_RSZ_OVF;

	if (sbl_pcr) {
		isp->isp_af.buf_err = 1;
		isp->isp_aewb.buf_err = 1;
		isp->isp_hist.buf_err = 1;
		dev_dbg(dev, "SBL overflow (PCR = 0x%08x)\n", sbl_pcr);
	}

	if (sbl_pcr & (ISPSBL_PCR_CCDC_WBL_OVF | ISPSBL_PCR_CSIA_WBL_OVF
		     | ISPSBL_PCR_CSIB_WBL_OVF)) {
		isp->isp_ccdc.error = 1;
		if (isp->isp_ccdc.output & CCDC_OUTPUT_PREVIEW)
			isp->isp_prev.error = 1;
		if (isp->isp_ccdc.output & CCDC_OUTPUT_RESIZER)
			isp->isp_res.error = 1;
	}

	if (sbl_pcr & ISPSBL_PCR_PRV_WBL_OVF) {
		isp->isp_prev.error = 1;
		if (isp->isp_res.input == RESIZER_INPUT_VP &&
		    !(isp->isp_ccdc.output & CCDC_OUTPUT_RESIZER))
			isp->isp_res.error = 1;
	}

	if (sbl_pcr & (ISPSBL_PCR_RSZ1_WBL_OVF
		       | ISPSBL_PCR_RSZ2_WBL_OVF
		       | ISPSBL_PCR_RSZ3_WBL_OVF
		       | ISPSBL_PCR_RSZ4_WBL_OVF))
		isp->isp_res.error = 1;

	if (sbl_pcr & ISPSBL_PCR_H3A_AF_WBL_OVF)
		isp->isp_af.buf_err = 1;

	if (sbl_pcr & ISPSBL_PCR_H3A_AEAWB_WBL_OVF)
		isp->isp_aewb.buf_err = 1;
}

/**
 * isp_isr - Interrupt Service Routine for Camera ISP module.
 * @irq: Not used currently.
 * @_pdev: Pointer to the platform device associated with the OMAP3 ISP.
 *
 * Handles the corresponding callback if plugged in.
 *
 * Returns IRQ_HANDLED when IRQ was correctly handled, or IRQ_NONE when the
 * IRQ wasn't handled.
 **/
static irqreturn_t isp_isr(int irq, void *_isp)
{
	struct isp_device *isp = _isp;
	struct device *dev = isp->dev;
	u32 irqstatus;
	int ret;

	irqstatus = isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
	isp_reg_writel(isp, irqstatus, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);

	isp_isr_sbl(isp);

	if (irqstatus & IRQ0STATUS_CSIA_IRQ) {
		ret = isp_csi2_isr(&isp->isp_csi2a);
		if (ret)
			isp->isp_ccdc.error = 1;
	}

	if (irqstatus & IRQ0STATUS_CSIB_IRQ) {
		ret = ispccp2_isr(isp);
		if (ret)
			isp->isp_ccdc.error = 1;
	}

	if (irqstatus & IRQ0STATUS_CCDC_LSC_PREF_ERR_IRQ) {
		isp->isp_ccdc.error = 1;
		ispccdc_lsc_error_handler(&isp->isp_ccdc);
		dev_dbg(dev, "lsc prefetch error\n");
	}

	if (irqstatus & IRQ0STATUS_CCDC_VD0_IRQ) {
		if (isp->isp_ccdc.output & CCDC_OUTPUT_PREVIEW)
			isppreview_isr_frame_sync(&isp->isp_prev);
		if (isp->isp_ccdc.output & CCDC_OUTPUT_RESIZER)
			ispresizer_isr_frame_sync(&isp->isp_res);
		ispccdc_isr(&isp->isp_ccdc);
	}

	if (irqstatus & IRQ0STATUS_HS_VS_IRQ)
		ispccdc_hs_vs_isr(&isp->isp_ccdc);

	if (irqstatus & IRQ0STATUS_PRV_DONE_IRQ) {
		if (isp->isp_prev.output & PREVIEW_OUTPUT_RESIZER)
			ispresizer_isr_frame_sync(&isp->isp_res);
		isppreview_isr(&isp->isp_prev);
	}

	if (irqstatus & IRQ0STATUS_RSZ_DONE_IRQ)
		ispresizer_isr(&isp->isp_res);

	if (irqstatus & IRQ0STATUS_H3A_AWB_DONE_IRQ)
		ispstat_isr(&isp->isp_aewb);

	if (irqstatus & IRQ0STATUS_H3A_AF_DONE_IRQ)
		ispstat_isr(&isp->isp_af);

	if (irqstatus & IRQ0STATUS_HIST_DONE_IRQ)
		ispstat_isr(&isp->isp_hist);

	isp_flush(isp);

#if defined(DEBUG) && defined(ISP_ISR_DEBUG)
	isp_isr_dbg(isp, irqstatus);
#endif

	return IRQ_HANDLED;
}

/* Device name, needed for resource tracking layer */
static struct device_driver camera_drv = {
	.name = "camera"
};

static struct device camera_dev = {
	.driver = &camera_drv,
};

/* -----------------------------------------------------------------------------
 * Pipeline management
 */

/*
 * isp_pipeline_enable - Enable streaming on a pipeline
 * @dev: OMAP3 ISP device
 * @video: ISP video node at the end of the pipeline
 * @mode: Stream mode (single shot or continuous)
 *
 * Walk the entities chain starting at the given output video node and start all
 * modules in the chain in the given mode.
 *
 * Return 0 if successfull, or the return value of the failed video::s_stream
 * operation otherwise.
 */
static int isp_pipeline_enable(struct isp_device *isp, struct isp_video *video,
			       enum isp_pipeline_stream_state mode)
{
	struct media_entity_pad *pad;
	struct media_entity *entity;
	struct v4l2_subdev *subdev;
	int ret = 0;

	entity = &video->video.entity;
	while (1) {
		pad = &entity->pads[0];
		if (pad->type != MEDIA_PAD_TYPE_INPUT)
			break;

		pad = media_entity_remote_pad(pad);
		if (pad == NULL ||
		    pad->entity->type != MEDIA_ENTITY_TYPE_SUBDEV)
			break;

		entity = pad->entity;
		subdev = media_entity_to_v4l2_subdev(entity);

		ret = v4l2_subdev_call(subdev, video, s_stream, mode);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			break;

		if (subdev == &isp->isp_ccdc.subdev) {
			v4l2_subdev_call(&isp->isp_aewb.subdev, video,
					s_stream, mode);
			v4l2_subdev_call(&isp->isp_af.subdev, video,
					s_stream, mode);
			v4l2_subdev_call(&isp->isp_hist.subdev, video,
					s_stream, mode);
		}
	}

	return ret;
}

static int isp_pipeline_wait_resizer(struct isp_device *isp)
{
	return ispresizer_busy(&isp->isp_res);
}

static int isp_pipeline_wait_preview(struct isp_device *isp)
{
	return isppreview_busy(&isp->isp_prev);
}

static int isp_pipeline_wait_ccdc(struct isp_device *isp)
{
	return ispstat_busy(&isp->isp_af)
	    || ispstat_busy(&isp->isp_aewb)
	    || ispstat_busy(&isp->isp_hist)
	    || ispccdc_busy(&isp->isp_ccdc);
}

#define ISP_STOP_TIMEOUT	msecs_to_jiffies(1000)

static int isp_pipeline_wait(struct isp_device *isp,
			     int(*busy)(struct isp_device *isp))
{
	unsigned long timeout = jiffies + ISP_STOP_TIMEOUT;

	while (!time_after(jiffies, timeout)) {
		if (!busy(isp))
			return 0;
	}

	return 1;
}

/*
 * isp_pipeline_disable - Disable streaming on a pipeline
 * @dev: OMAP3 ISP device
 * @video: ISP video node at the end of the pipeline
 *
 * Walk the entities chain starting at the given output video node and stop all
 * modules in the chain. Wait synchronously for the modules to be stopped if
 * necessary.
 *
 * Return 0 if all modules have been properly stopped, or -ETIMEDOUT if a module
 * can't be stopped (in which case a software reset of the ISP is probably
 * necessary).
 */
static int isp_pipeline_disable(struct isp_device *isp, struct isp_video *video)
{
	struct media_entity_pad *pad;
	struct media_entity *entity;
	struct v4l2_subdev *subdev;
	int failure = 0;
	int ret;

	/*
	 * We need to stop all the modules after CCDC first or they'll
	 * never stop since they may not get a full frame from CCDC.
	 */
	entity = &video->video.entity;
	while (1) {
		pad = &entity->pads[0];
		if (pad->type != MEDIA_PAD_TYPE_INPUT)
			break;

		pad = media_entity_remote_pad(pad);
		if (pad == NULL ||
		    pad->entity->type != MEDIA_ENTITY_TYPE_SUBDEV)
			break;

		entity = pad->entity;
		subdev = media_entity_to_v4l2_subdev(entity);

		v4l2_subdev_call(subdev, video, s_stream, 0);

		if (subdev == &isp->isp_res.subdev) {
			ret = isp_pipeline_wait(isp, isp_pipeline_wait_resizer);
		} else if (subdev == &isp->isp_prev.subdev) {
			ret = isp_pipeline_wait(isp, isp_pipeline_wait_preview);
		} else if (subdev == &isp->isp_ccdc.subdev) {
			v4l2_subdev_call(&isp->isp_aewb.subdev,
					 video, s_stream, 0);
			v4l2_subdev_call(&isp->isp_af.subdev,
					 video, s_stream, 0);
			v4l2_subdev_call(&isp->isp_hist.subdev,
					 video, s_stream, 0);
			ret = isp_pipeline_wait(isp, isp_pipeline_wait_ccdc);
		} else {
			ret = 0;
		}

		if (ret) {
			dev_info(isp->dev, "Unable to stop %s\n", subdev->name);
			failure = -ETIMEDOUT;
		}
	}

	return failure;
}

/*
 * isp_pipeline_set_stream - Enable/disable streaming on a pipeline
 * @dev: OMAP3 ISP device
 * @video: ISP video node at the end of the pipeline
 * @state: Stream state (stopped, single shot or continuous)
 *
 * Set the pipeline to the given stream state. Pipelines can be started in
 * single-shot or continuous mode.
 *
 * Return 0 if successfull, or the return value of the failed video::s_stream
 * operation otherwise.
 */
int isp_pipeline_set_stream(struct isp_device *isp, struct isp_video *video,
			    enum isp_pipeline_stream_state state)
{
	if (state == ISP_PIPELINE_STREAM_STOPPED)
		return isp_pipeline_disable(isp, video);
	else
		return isp_pipeline_enable(isp, video, state);
}

/**
 * __isp_disable_modules - Disable ISP submodules with a timeout to be idle.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @suspend: If 0, disable modules; if 1, send modules to suspend state.
 *
 * Returns 0 if stop/suspend left in idle state all the submodules properly,
 * or returns 1 if a general Reset is required to stop/suspend the submodules.
 **/
static int __isp_disable_modules(struct isp_device *isp, int suspend)
{
	unsigned long timeout;
	int reset = 0;

	/*
	 * We need to stop all the modules after CCDC first or they'll
	 * never stop since they may not get a full frame from CCDC.
	 */
	if (suspend) {
		ispstat_suspend(&isp->isp_af);
		ispstat_suspend(&isp->isp_aewb);
		ispstat_suspend(&isp->isp_hist);
	} else {
		v4l2_subdev_call(&isp->isp_aewb.subdev, video, s_stream, 0);
		v4l2_subdev_call(&isp->isp_af.subdev, video, s_stream, 0);
		v4l2_subdev_call(&isp->isp_hist.subdev, video, s_stream, 0);
	}

	v4l2_subdev_call(&isp->isp_res.subdev, video, s_stream, 0);
	v4l2_subdev_call(&isp->isp_prev.subdev, video, s_stream, 0);

	timeout = jiffies + ISP_STOP_TIMEOUT;
	while (ispstat_busy(&isp->isp_af)
	       || ispstat_busy(&isp->isp_aewb)
	       || ispstat_busy(&isp->isp_hist)
	       || isppreview_busy(&isp->isp_prev)
	       || ispresizer_busy(&isp->isp_res)) {
		if (time_after(jiffies, timeout)) {
			dev_info(isp->dev, "can't stop non-ccdc modules.\n");
			reset = 1;
			break;
		}
		msleep(1);
	}

	/* Let's stop CCDC now. */
	v4l2_subdev_call(&isp->isp_ccdc.subdev, video, s_stream, 0);

	timeout = jiffies + ISP_STOP_TIMEOUT;
	while (ispccdc_busy(&isp->isp_ccdc)) {
		if (time_after(jiffies, timeout)) {
			dev_info(isp->dev, "can't stop ccdc module.\n");
			reset = 1;
			break;
		}
		msleep(1);
	}

	v4l2_subdev_call(&isp->isp_csi2a.subdev, video, s_stream, 0);
	v4l2_subdev_call(&isp->isp_ccp2.subdev, video, s_stream, 0);

	return reset;
}

/**
 * isp_suspend_modules - Suspend ISP submodules.
 * @dev: Device pointer specific to the OMAP3 ISP.
 *
 * Returns 0 if suspend left in idle state all the submodules properly,
 * or returns 1 if a general Reset is required to suspend the submodules.
 **/
static int isp_suspend_modules(struct isp_device *isp)
{
	return __isp_disable_modules(isp, 1);
}

/**
 * isp_resume_modules - Resume ISP submodules.
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
static void isp_resume_modules(struct isp_device *isp)
{
	ispstat_resume(&isp->isp_hist);
	ispstat_resume(&isp->isp_aewb);
	ispstat_resume(&isp->isp_af);
}

/**
 * isp_reset - Reset ISP with a timeout wait for idle.
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
static void isp_reset(struct isp_device *isp)
{
	unsigned long timeout = 0;

	isp_reg_writel(isp,
		       isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG)
		       | ISP_SYSCONFIG_SOFTRESET,
		       OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG);
	while (!(isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN,
			       ISP_SYSSTATUS) & 0x1)) {
		if (timeout++ > 10000) {
			dev_alert(isp->dev, "cannot reset ISP\n");
			break;
		}
		udelay(1);
	}
}

/**
 * isp_stop - Stop ISP.
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
void isp_stop(struct isp_device *isp, struct isp_video *video)
{
	if (!isp_pipeline_disable(isp, video))
		return;

	isp_save_ctx(isp);
	isp_reset(isp);
	isp_restore_ctx(isp);
}
EXPORT_SYMBOL(isp_stop);

/**
 * isp_handle_private - Handle all private ioctls for isp module.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @cmd: ioctl cmd value
 * @arg: ioctl arg value
 *
 * Return 0 if successful, -EINVAL if chosen cmd value is not handled or value
 * is out of bounds, -EFAULT if ioctl arg value is not valid.
 * Function simply routes the input ioctl cmd id to the appropriate handler in
 * the isp module.
 **/
int isp_handle_private(struct isp_device *isp, int cmd, void *arg)
{
	int rval = 0;

	switch (cmd) {
	case VIDIOC_PRIVATE_ISP_CCDC_CFG:
		rval = v4l2_subdev_call(&isp->isp_ccdc.subdev, core, ioctl,
					cmd, arg);
		break;
	case VIDIOC_PRIVATE_ISP_PRV_CFG:
		rval = v4l2_subdev_call(&isp->isp_prev.subdev, core, ioctl,
					cmd, arg);
		break;
	default:
		rval = -EINVAL;
		break;
	}
	return rval;
}
EXPORT_SYMBOL(isp_handle_private);

/**
 * isp_save_ctx - Saves ISP, CCDC, HIST, H3A, PREV, RESZ & MMU context.
 * @dev: Device pointer specific to the OMAP3 ISP.
 *
 * Routine for saving the context of each module in the ISP.
 * CCDC, HIST, H3A, PREV, RESZ and MMU.
 **/
static void isp_save_ctx(struct isp_device *isp)
{
	isp_save_context(isp, isp_reg_list);
	ispccp2_save_context(isp);
	ispccdc_save_context(isp);
	if (isp->iommu)
		iommu_save_ctx(isp->iommu);
	isppreview_save_context(isp);
	ispresizer_save_context(isp);
}

/**
 * isp_restore_ctx - Restores ISP, CCDC, HIST, H3A, PREV, RESZ & MMU context.
 * @dev: Device pointer specific to the OMAP3 ISP.
 *
 * Routine for restoring the context of each module in the ISP.
 * CCDC, HIST, H3A, PREV, RESZ and MMU.
 **/
static void isp_restore_ctx(struct isp_device *isp)
{
	isp_restore_context(isp, isp_reg_list);
	ispccp2_restore_context(isp);
	ispccdc_restore_context(isp);
	if (isp->iommu)
		iommu_restore_ctx(isp->iommu);
	isppreview_restore_context(isp);
	ispresizer_restore_context(isp);
}

/* -----------------------------------------------------------------------------
 * SBL resources management
 */
#define OMAP3_ISP_SBL_READ	(OMAP3_ISP_SBL_CSI1_READ | \
				 OMAP3_ISP_SBL_CCDC_LSC_READ | \
				 OMAP3_ISP_SBL_PREVIEW_READ | \
				 OMAP3_ISP_SBL_RESIZER_READ)
#define OMAP3_ISP_SBL_WRITE	(OMAP3_ISP_SBL_CSI1_WRITE | \
				 OMAP3_ISP_SBL_CSI2A_WRITE | \
				 OMAP3_ISP_SBL_CSI2C_WRITE | \
				 OMAP3_ISP_SBL_CCDC_WRITE | \
				 OMAP3_ISP_SBL_PREVIEW_WRITE)

void isp_sbl_enable(struct isp_device *isp, enum isp_sbl_resource res)
{
	u32 sbl = 0;

	isp->sbl_resources |= res;

	if (isp->sbl_resources & OMAP3_ISP_SBL_CSI1_READ)
		sbl |= ISPCTRL_SBL_SHARED_RPORTA;

	if (isp->sbl_resources & OMAP3_ISP_SBL_CCDC_LSC_READ)
		sbl |= ISPCTRL_SBL_SHARED_RPORTB;

	if (isp->sbl_resources & OMAP3_ISP_SBL_CSI2C_WRITE)
		sbl |= ISPCTRL_SBL_SHARED_WPORTC;

	if (isp->sbl_resources & OMAP3_ISP_SBL_RESIZER_WRITE)
		sbl |= ISPCTRL_SBL_WR0_RAM_EN;

	if (isp->sbl_resources & OMAP3_ISP_SBL_WRITE)
		sbl |= ISPCTRL_SBL_WR1_RAM_EN;

	if (isp->sbl_resources & OMAP3_ISP_SBL_READ)
		sbl |= ISPCTRL_SBL_RD_RAM_EN;

	isp_reg_or(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL, sbl);
}

void isp_sbl_disable(struct isp_device *isp, enum isp_sbl_resource res)
{
	u32 sbl = 0;

	isp->sbl_resources &= ~res;

	if (!(isp->sbl_resources & OMAP3_ISP_SBL_CSI1_READ))
		sbl |= ISPCTRL_SBL_SHARED_RPORTA;

	if (!(isp->sbl_resources & OMAP3_ISP_SBL_CCDC_LSC_READ))
		sbl |= ISPCTRL_SBL_SHARED_RPORTB;

	if (!(isp->sbl_resources & OMAP3_ISP_SBL_CSI2C_WRITE))
		sbl |= ISPCTRL_SBL_SHARED_WPORTC;

	if (!(isp->sbl_resources & OMAP3_ISP_SBL_RESIZER_WRITE))
		sbl |= ISPCTRL_SBL_WR0_RAM_EN;

	if (!(isp->sbl_resources & OMAP3_ISP_SBL_WRITE))
		sbl |= ISPCTRL_SBL_WR1_RAM_EN;

	if (!(isp->sbl_resources & OMAP3_ISP_SBL_READ))
		sbl |= ISPCTRL_SBL_RD_RAM_EN;

	isp_reg_and(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL, ~sbl);
}

/* --------------------------------------------------------------------------
 * Clock management
 */

/**
 * isp_enable_clocks - Enable ISP clocks
 * @dev: Device pointer specific to the OMAP3 ISP.
 *
 * Return 0 if successful, or clk_enable return value if any of tthem fails.
 **/
static int isp_enable_clocks(struct isp_device *isp)
{
	int r;
	unsigned long rate;
	int divisor;

	/*
	 * cam_mclk clock chain:
	 *   dpll4 -> dpll4_m5 -> dpll4_m5x2 -> cam_mclk
	 *
	 * In OMAP3630 dpll4_m5x2 != 2 x dpll4_m5 but both are
	 * set to the same value. Hence the rate set for dpll4_m5
	 * has to be twice of what is set on OMAP3430 to get
	 * the required value for cam_mclk
	 */
	if (cpu_is_omap3630())
		divisor = 1;
	else
		divisor = 2;

	r = clk_enable(isp->clock[ISP_CLK_CAM_ICK]);
	if (r) {
		dev_err(isp->dev, "clk_enable cam_ick failed\n");
		goto out_clk_enable_ick;
	}
	r = clk_set_rate(isp->clock[ISP_CLK_DPLL4_M5_CK],
			 CM_CAM_MCLK_HZ/divisor);
	if (r) {
		dev_err(isp->dev, "clk_set_rate for dpll4_m5_ck failed\n");
		goto out_clk_enable_mclk;
	}
	r = clk_enable(isp->clock[ISP_CLK_CAM_MCLK]);
	if (r) {
		dev_err(isp->dev, "clk_enable cam_mclk failed\n");
		goto out_clk_enable_mclk;
	}
	rate = clk_get_rate(isp->clock[ISP_CLK_CAM_MCLK]);
	if (rate != CM_CAM_MCLK_HZ)
		dev_warn(isp->dev, "unexpected cam_mclk rate:\n"
				   " expected : %d\n"
				   " actual   : %ld\n", CM_CAM_MCLK_HZ, rate);
	r = clk_enable(isp->clock[ISP_CLK_CSI2_FCK]);
	if (r) {
		dev_err(isp->dev, "clk_enable csi2_fck failed\n");
		goto out_clk_enable_csi2_fclk;
	}
	return 0;

out_clk_enable_csi2_fclk:
	clk_disable(isp->clock[ISP_CLK_CAM_MCLK]);
out_clk_enable_mclk:
	clk_disable(isp->clock[ISP_CLK_CAM_ICK]);
out_clk_enable_ick:
	return r;
}

/**
 * isp_disable_clocks - Disable ISP clocks
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
static void isp_disable_clocks(struct isp_device *isp)
{
	clk_disable(isp->clock[ISP_CLK_CAM_ICK]);
	clk_disable(isp->clock[ISP_CLK_CAM_MCLK]);
	clk_disable(isp->clock[ISP_CLK_CSI2_FCK]);
}

static const char *isp_clocks[] = {
	"cam_ick",
	"cam_mclk",
	"dpll4_m5_ck",
	"csi2_96m_fck",
	"l3_ick",
};

static void isp_put_clocks(struct isp_device *isp)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(isp_clocks); ++i) {
		if (isp->clock[i]) {
			clk_put(isp->clock[i]);
			isp->clock[i] = NULL;
		}
	}
}

static int isp_get_clocks(struct isp_device *isp)
{
	struct clk *clk;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(isp_clocks); ++i) {
		clk = clk_get(&camera_dev, isp_clocks[i]);
		if (IS_ERR(clk)) {
			dev_err(isp->dev, "clk_get %s failed\n", isp_clocks[i]);
			isp_put_clocks(isp);
			return PTR_ERR(clk);
		}

		isp->clock[i] = clk;
	}

	return 0;
}

/*
 * isp_get - Acquire the ISP resource.
 *
 * Initializes the clocks for the first acquire.
 *
 * Increment the reference count on the ISP. If the first reference is taken,
 * enable power regulators and clocks and power-up all submodules.
 *
 * Return a pointer to the ISP device structure, or NULL if an error occured.
 */
struct isp_device *isp_get(struct isp_device *isp)
{
	struct isp_device *__isp = NULL;

	if (isp == NULL)
		return NULL;

	mutex_lock(&isp->isp_mutex);
	if (++isp->ref_count != 1) {
		__isp = isp;
		goto out;
	}

	if (regulator_enable(isp->vdd_csiphy1) < 0)
		goto out;

	if (regulator_enable(isp->vdd_csiphy2) < 0) {
		regulator_disable(isp->vdd_csiphy1);
		goto out;
	}

	if (isp_enable_clocks(isp) < 0) {
		regulator_disable(isp->vdd_csiphy1);
		regulator_disable(isp->vdd_csiphy2);
		goto out;
	}

	/* We don't want to restore context before saving it! */
	if (isp->has_context)
		isp_restore_ctx(isp);
	else
		isp->has_context = 1;

	isp_enable_interrupts(isp);
	__isp = isp;

out:
	mutex_unlock(&isp->isp_mutex);

	return __isp;
}
EXPORT_SYMBOL(isp_get);

/*
 * isp_put - Release the ISP
 *
 * Decrement the reference count on the ISP. If the last reference is released,
 * power-down all submodules, disable clocks, free temporary buffers and disable
 * power regulators.
 */
void isp_put(struct isp_device *isp)
{
	if (isp == NULL)
		return;

	mutex_lock(&isp->isp_mutex);
	if (--isp->ref_count == 0) {
		isp_disable_interrupts(isp);
		isp_save_ctx(isp);
		isp_disable_clocks(isp);
		regulator_disable(isp->vdd_csiphy1);
		regulator_disable(isp->vdd_csiphy2);
	}
	mutex_unlock(&isp->isp_mutex);
}
EXPORT_SYMBOL(isp_put);

/**
 * isp_save_context - Saves the values of the ISP module registers.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @reg_list: Structure containing pairs of register address and value to
 *            modify on OMAP.
 **/
void isp_save_context(struct isp_device *isp, struct isp_reg *reg_list)
{
	struct isp_reg *next = reg_list;

	for (; next->reg != ISP_TOK_TERM; next++)
		next->val = isp_reg_readl(isp, next->mmio_range, next->reg);
}

/**
 * isp_restore_context - Restores the values of the ISP module registers.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @reg_list: Structure containing pairs of register address and value to
 *            modify on OMAP.
 **/
void isp_restore_context(struct isp_device *isp, struct isp_reg *reg_list)
{
	struct isp_reg *next = reg_list;

	for (; next->reg != ISP_TOK_TERM; next++)
		isp_reg_writel(isp, next->val, next->mmio_range, next->reg);
}

/* --------------------------------------------------------------------------
 * Platform device driver
 */

/**
 * isp_remove - Remove ISP platform device
 * @pdev: Pointer to ISP platform device
 *
 * Always returns 0.
 **/
static int isp_remove(struct platform_device *pdev)
{
	struct isp_device *isp = platform_get_drvdata(pdev);
	int i;

	isp_ccp2_cleanup(isp);
	isp_csi2_cleanup(isp);
	isp_get(isp);
	if (isp->iommu)
		iommu_put(isp->iommu);
	isp_put(isp);
	isp_resizer_cleanup(isp);
	isp_preview_cleanup(isp);
	isph3a_aewb_cleanup(isp);
	isph3a_af_cleanup(isp);
	isphist_cleanup(isp);
	isp_ccdc_cleanup(isp);

	isp_put_clocks(isp);

	free_irq(isp->irq_num, isp);

	for (i = 0; i < OMAP3_ISP_IOMEM_LAST; i++) {
		if (isp->mmio_base[i]) {
			iounmap(isp->mmio_base[i]);
			isp->mmio_base[i] = NULL;
		}

		if (isp->mmio_base_phys[i]) {
			release_mem_region(isp->mmio_base_phys[i],
					   isp->mmio_size[i]);
			isp->mmio_base_phys[i] = 0;
		}
	}

	regulator_put(isp->vdd_csiphy1);
	regulator_put(isp->vdd_csiphy2);
	kfree(isp);

	return 0;
}

#ifdef CONFIG_PM

/**
 * isp_suspend - Suspend routine for the ISP
 * @dev: Pointer to ISP device
 *
 * Always returns 0.
 **/
static int isp_suspend(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	int reset;

	WARN_ON(mutex_is_locked(&isp->isp_mutex));

	if (isp->ref_count == 0)
		goto out;

	isp_disable_interrupts(isp);
	reset = isp_suspend_modules(isp);
	isp_save_ctx(isp);
	if (reset)
		isp_reset(isp);

	isp_disable_clocks(isp);

out:
	return 0;
}

/**
 * isp_resume - Resume routine for the ISP
 * @dev: Pointer to ISP device
 *
 * Returns 0 if successful, or isp_enable_clocks return value otherwise.
 **/
static int isp_resume(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	int ret_err = 0;

	if (isp->ref_count == 0)
		goto out;

	ret_err = isp_enable_clocks(isp);
	if (ret_err)
		goto out;
	isp_restore_ctx(isp);
	isp_enable_interrupts(isp);
	isp_resume_modules(isp);

out:
	return ret_err;
}

#else

#define isp_suspend	NULL
#define isp_resume	NULL

#endif /* CONFIG_PM */

static int isp_map_mem_resource(struct platform_device *pdev,
				struct isp_device *isp,
				enum isp_mem_resources res)
{
	struct resource *mem;

	/* request the mem region for the camera registers */

	mem = platform_get_resource(pdev, IORESOURCE_MEM, res);
	if (!mem) {
		dev_err(isp->dev, "no mem resource?\n");
		return -ENODEV;
	}

	if (!request_mem_region(mem->start, mem->end - mem->start + 1,
				pdev->name)) {
		dev_err(isp->dev,
			"cannot reserve camera register I/O region\n");
		return -ENODEV;
	}
	isp->mmio_base_phys[res] = mem->start;
	isp->mmio_size[res] = mem->end - mem->start + 1;

	/* map the region */
	isp->mmio_base[res] = ioremap_nocache(isp->mmio_base_phys[res],
					      isp->mmio_size[res]);
	if (!isp->mmio_base[res]) {
		dev_err(isp->dev, "cannot map camera register I/O region\n");
		return -ENODEV;
	}

	return 0;
}

/**
 * isp_probe - Probe ISP platform device
 * @pdev: Pointer to ISP platform device
 *
 * Returns 0 if successful,
 *   -ENOMEM if no memory available,
 *   -ENODEV if no platform device resources found
 *     or no space for remapping registers,
 *   -EINVAL if couldn't install ISR,
 *   or clk_get return error value.
 **/
static int isp_probe(struct platform_device *pdev)
{
	struct isp_platform_data *pdata = pdev->dev.platform_data;
	struct isp_device *isp;
	int ret_err = 0;
	int i, m;

	if (pdata == NULL)
		return -EINVAL;

	isp = kzalloc(sizeof(*isp), GFP_KERNEL);
	if (!isp) {
		dev_err(&pdev->dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	isp->pdata = pdata;

	isp->vdd_csiphy1 = regulator_get(&pdev->dev, "VDD_CSIPHY1");
	if (IS_ERR(isp->vdd_csiphy1)) {
		dev_err(&pdev->dev,
			"failed to get regulator for vdd_csiphy1\n");
		ret_err = -ENODEV;
		goto out_vdd_csiphy1_fail;
	}

	isp->vdd_csiphy2 = regulator_get(&pdev->dev, "VDD_CSIPHY2");
	if (IS_ERR(isp->vdd_csiphy2)) {
		dev_err(&pdev->dev,
			"failed to get regulator for vdd_csiphy2\n");
		ret_err = -ENODEV;
		goto out_vdd_csiphy2_fail;
	}

	platform_set_drvdata(pdev, isp);

	isp->dev = &pdev->dev;

	ret_err = isp_map_mem_resource(pdev, isp, OMAP3_ISP_IOMEM_MAIN);
	if (ret_err)
		goto out_iomem_main_fail;

	ret_err = isp_get_clocks(isp);
	if (ret_err < 0)
		goto out_free_mmio;

	isp->ref_count = 0;

	mutex_init(&isp->isp_mutex);
	spin_lock_init(&isp->stat_lock);

	isp->raw_dmamask = DMA_BIT_MASK(32);
	isp->dev->dma_mask = &isp->raw_dmamask;
	isp->dev->coherent_dma_mask = DMA_BIT_MASK(32);

	if (!isp_get(isp))
		goto out_free_mmio;

	/* Get ISP revision */
	isp->revision = isp_reg_readl(isp,
				      OMAP3_ISP_IOMEM_MAIN, ISP_REVISION);
	dev_info(isp->dev, "Revision %d.%d found\n",
		 (isp->revision & 0xF0) >> 4, isp->revision & 0xF);

	for (m = 0; m < ARRAY_SIZE(isp_res_maps); m++)
		if (isp->revision == isp_res_maps[m].isp_rev)
			break;

	if (m == ARRAY_SIZE(isp_res_maps)) {
		dev_err(isp->dev, "no resource map found for ISP rev %d.%d\n",
			(isp->revision & 0xf0) >> 4, isp->revision & 0xf);
		ret_err = -ENODEV;
		goto out_free_mmio;
	}

	for (i = 1; i < OMAP3_ISP_IOMEM_LAST; i++) {
		if (isp_res_maps[m].map & 1 << i) {
			ret_err = isp_map_mem_resource(pdev, isp, i);
			if (ret_err)
				goto out_free_mmio;
		}
	}

	isp->irq_num = platform_get_irq(pdev, 0);
	if (isp->irq_num <= 0) {
		dev_err(isp->dev, "no irq for camera?\n");
		ret_err = -ENODEV;
		goto out_free_mmio;
	}

	if (request_irq(isp->irq_num, isp_isr, IRQF_SHARED,
			"Omap 3 Camera ISP", isp)) {
		dev_err(isp->dev, "could not install isr\n");
		ret_err = -EINVAL;
		goto out_request_irq;
	}

	isp->iommu = iommu_get("isp");
	if (IS_ERR(isp->iommu)) {
		ret_err = PTR_ERR(isp->iommu);
		isp->iommu = NULL;
	}
	if (!isp->iommu) {
		isp_put(isp);
		goto out_iommu_get;
	}

	ret_err = isp_ccdc_init(isp);
	if (ret_err)
		dev_err(isp->dev, "could not init ispccdc\n");
	ret_err = isphist_init(isp);
	if (ret_err)
		dev_err(isp->dev, "could not init isphist\n");
	ret_err = isph3a_aewb_init(isp);
	if (ret_err)
		dev_err(isp->dev, "could not init isph3a_aewb\n");
	ret_err = isph3a_af_init(isp);
	if (ret_err)
		dev_err(isp->dev, "could not init isph3a_af\n");
	ret_err = isp_preview_init(isp);
	if (ret_err)
		dev_err(isp->dev, "could not init isppreview\n");
	ret_err = isp_resizer_init(isp);
	if (ret_err)
		dev_err(isp->dev, "could not init ispresizer\n");
	ret_err = isp_csiphy_init(isp);
	if (ret_err)
		dev_err(isp->dev, "could not init ispcsiphy\n");
	ret_err = isp_csi2_init(isp);
	if (ret_err)
		dev_err(isp->dev, "could not init ispcsi2\n");
	ret_err = isp_ccp2_init(isp);
	if (ret_err)
		dev_err(isp->dev, "could not init ispccp2\n");

	isp_power_settings(isp, 1);
	isp_put(isp);

	/* Connect the submodules. */
	ret_err = media_entity_create_link(
			&isp->isp_csi2a.subdev.entity, CSI2_PAD_SOURCE,
			&isp->isp_ccdc.subdev.entity, CCDC_PAD_SINK, 0);
	if (ret_err < 0)
		goto out_iommu_get;

	ret_err = media_entity_create_link(
			&isp->isp_ccp2.subdev.entity, CCP2_PAD_SOURCE,
			&isp->isp_ccdc.subdev.entity, CCDC_PAD_SINK, 0);
	if (ret_err < 0)
		goto out_iommu_get;

	ret_err = media_entity_create_link(
			&isp->isp_ccdc.subdev.entity, CCDC_PAD_SOURCE_VP,
			&isp->isp_prev.subdev.entity, PREV_PAD_SINK, 0);
	if (ret_err < 0)
		goto out_iommu_get;

	ret_err = media_entity_create_link(
			&isp->isp_ccdc.subdev.entity, CCDC_PAD_SOURCE_OF,
			&isp->isp_res.subdev.entity, RESZ_PAD_SINK, 0);
	if (ret_err < 0)
		goto out_iommu_get;

	ret_err = media_entity_create_link(
			&isp->isp_prev.subdev.entity, PREV_PAD_SOURCE,
			&isp->isp_res.subdev.entity, RESZ_PAD_SINK, 0);
	if (ret_err < 0)
		goto out_iommu_get;

	ret_err = media_entity_create_link(
			&isp->isp_ccdc.subdev.entity, CCDC_PAD_SOURCE_VP,
			&isp->isp_aewb.subdev.entity, 0, 0);
	if (ret_err < 0)
		goto out_iommu_get;

	ret_err = media_entity_create_link(
			&isp->isp_ccdc.subdev.entity, CCDC_PAD_SOURCE_VP,
			&isp->isp_af.subdev.entity, 0, 0);
	if (ret_err < 0)
		goto out_iommu_get;

	ret_err = media_entity_create_link(
			&isp->isp_ccdc.subdev.entity, CCDC_PAD_SOURCE_VP,
			&isp->isp_hist.subdev.entity, 0, 0);

	if (ret_err < 0)
		goto out_iommu_get;

	return 0;

out_iommu_get:
	free_irq(isp->irq_num, isp);
out_request_irq:
	isp_put_clocks(isp);
out_free_mmio:
	for (i = 0; i < OMAP3_ISP_IOMEM_LAST; i++) {
		if (isp->mmio_base[i]) {
			iounmap(isp->mmio_base[i]);
			isp->mmio_base[i] = NULL;
		}

		if (isp->mmio_base_phys[i]) {
			release_mem_region(isp->mmio_base_phys[i],
					   isp->mmio_size[i]);
			isp->mmio_base_phys[i] = 0;
		}
	}
out_iomem_main_fail:
	regulator_put(isp->vdd_csiphy2);
out_vdd_csiphy2_fail:
	regulator_put(isp->vdd_csiphy1);
out_vdd_csiphy1_fail:
	kfree(isp);
	platform_set_drvdata(pdev, NULL);
	return ret_err;
}

static struct dev_pm_ops omap3isp_pm_ops = {
	.suspend = isp_suspend,
	.resume  = isp_resume,
};

static struct platform_driver omap3isp_driver = {
	.probe = isp_probe,
	.remove = isp_remove,
	.driver = {
		.name = "omap3isp",
		.pm	= &omap3isp_pm_ops,
	},
};

/**
 * isp_init - ISP module initialization.
 **/
static int __init isp_init(void)
{
	return platform_driver_register(&omap3isp_driver);
}

/**
 * isp_cleanup - ISP module cleanup.
 **/
static void __exit isp_cleanup(void)
{
	platform_driver_unregister(&omap3isp_driver);
}

/**
 * isp_print_status - Prints the values of the ISP Control Module registers
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
void isp_print_status(struct isp_device *isp)
{
	dev_dbg(isp->dev, "###ISP_CTRL=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL));
	dev_dbg(isp->dev, "###ISP_TCTRL_CTRL=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_CTRL));
	dev_dbg(isp->dev, "###ISP_SYSCONFIG=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG));
	dev_dbg(isp->dev, "###ISP_SYSSTATUS=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_SYSSTATUS));
	dev_dbg(isp->dev, "###ISP_IRQ0ENABLE=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE));
	dev_dbg(isp->dev, "###ISP_IRQ0STATUS=0x%x\n",
		isp_reg_readl(isp, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS));
}

void omap3isp_unregister_entities(struct platform_device *pdev)
{
	struct isp_device *isp = platform_get_drvdata(pdev);

	if (isp == NULL)
		return;

	isp_csi2_unregister_entities(&isp->isp_csi2a);
	isp_ccp2_unregister_entities(&isp->isp_ccp2);
	isp_ccdc_unregister_entities(&isp->isp_ccdc);
	isp_preview_unregister_entities(&isp->isp_prev);
	isp_resizer_unregister_entities(&isp->isp_res);
	ispstat_unregister_entities(&isp->isp_aewb);
	ispstat_unregister_entities(&isp->isp_af);
	ispstat_unregister_entities(&isp->isp_hist);
}
EXPORT_SYMBOL(omap3isp_unregister_entities);

int omap3isp_register_entities(struct platform_device *pdev,
			       struct v4l2_device *v4l2_dev)
{
	struct isp_device *isp;
	int ret = 0;

	isp = platform_get_drvdata(pdev);
	if (isp == NULL) {
		ret = -ENODEV;
		goto done;
	}

	ret = isp_ccp2_register_entities(&isp->isp_ccp2, v4l2_dev);
	if (ret < 0)
		goto done;

	ret = isp_csi2_register_entities(&isp->isp_csi2a, v4l2_dev);
	if (ret < 0)
		goto done;

	ret = isp_ccdc_register_entities(&isp->isp_ccdc, v4l2_dev);
	if (ret < 0)
		goto done;

	ret = isp_preview_register_entities(&isp->isp_prev, v4l2_dev);
	if (ret < 0)
		goto done;

	ret = isp_resizer_register_entities(&isp->isp_res, v4l2_dev);
	if (ret < 0)
		goto done;

	ret = ispstat_register_entities(&isp->isp_aewb, v4l2_dev);
	if (ret < 0)
		goto done;

	ret = ispstat_register_entities(&isp->isp_af, v4l2_dev);
	if (ret < 0)
		goto done;

	ret = ispstat_register_entities(&isp->isp_hist, v4l2_dev);

done:
	if (ret)
		omap3isp_unregister_entities(pdev);

	return ret;
}
EXPORT_SYMBOL_GPL(omap3isp_register_entities);

module_init(isp_init);
module_exit(isp_cleanup);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("ISP Control Module Library");
MODULE_LICENSE("GPL");
