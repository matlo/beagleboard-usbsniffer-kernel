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
#include <linux/platform_device.h>
#include <linux/device.h>

#include "isp.h"
#include "ispreg.h"
#include "ispccdc.h"
#include "isph3a.h"
#include "isphist.h"
#include "isp_af.h"
#include "isppreview.h"
#include "ispresizer.h"
#include "ispcsi2.h"

static struct platform_device *omap3isp_pdev;

static void isp_save_ctx(struct device *dev);

static void isp_restore_ctx(struct device *dev);

static void isp_buf_init(struct device *dev);

/* List of image formats supported via OMAP ISP */
const static struct v4l2_fmtdesc isp_formats[] = {
	{
		.description = "UYVY, packed",
		.pixelformat = V4L2_PIX_FMT_UYVY,
	},
	{
		.description = "YUYV (YUV 4:2:2), packed",
		.pixelformat = V4L2_PIX_FMT_YUYV,
	},
	{
		.description = "Bayer10 (GrR/BGb)",
		.pixelformat = V4L2_PIX_FMT_SGRBG10,
	},
};

/**
 * struct vcontrol - Video control structure.
 * @qc: V4L2 Query control structure.
 * @current_value: Current value of the control.
 */
static struct vcontrol {
	struct v4l2_queryctrl qc;
	int current_value;
} video_control[] = {
	{
		{
			.id = V4L2_CID_BRIGHTNESS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Brightness",
			.minimum = ISPPRV_BRIGHT_LOW,
			.maximum = ISPPRV_BRIGHT_HIGH,
			.step = ISPPRV_BRIGHT_STEP,
			.default_value = ISPPRV_BRIGHT_DEF,
		},
		.current_value = ISPPRV_BRIGHT_DEF,
	},
	{
		{
			.id = V4L2_CID_CONTRAST,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Contrast",
			.minimum = ISPPRV_CONTRAST_LOW,
			.maximum = ISPPRV_CONTRAST_HIGH,
			.step = ISPPRV_CONTRAST_STEP,
			.default_value = ISPPRV_CONTRAST_DEF,
		},
		.current_value = ISPPRV_CONTRAST_DEF,
	},
	{
		{
			.id = V4L2_CID_COLORFX,
			.type = V4L2_CTRL_TYPE_MENU,
			.name = "Color Effects",
			.minimum = V4L2_COLORFX_NONE,
			.maximum = V4L2_COLORFX_SEPIA,
			.step = 1,
			.default_value = V4L2_COLORFX_NONE,
		},
		.current_value = V4L2_COLORFX_NONE,
	}
};

static struct v4l2_querymenu video_menu[] = {
	{
		.id = V4L2_CID_COLORFX,
		.index = 0,
		.name = "None",
	},
	{
		.id = V4L2_CID_COLORFX,
		.index = 1,
		.name = "B&W",
	},
	{
		.id = V4L2_CID_COLORFX,
		.index = 2,
		.name = "Sepia",
	},
};

/* Structure for saving/restoring ISP module registers */
static struct isp_reg isp_reg_list[] = {
	{OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE, 0},
	{OMAP3_ISP_IOMEM_MAIN, ISP_IRQ1ENABLE, 0},
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
void isp_flush(struct device *dev)
{
	isp_reg_writel(dev, 0, OMAP3_ISP_IOMEM_MAIN, ISP_REVISION);
	isp_reg_readl(dev, OMAP3_ISP_IOMEM_MAIN, ISP_REVISION);
}

/**
 * isp_reg_readl - Read value of an OMAP3 ISP register
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @isp_mmio_range: Range to which the register offset refers to.
 * @reg_offset: Register offset to read from.
 *
 * Returns an unsigned 32 bit value with the required register contents.
 **/
u32 isp_reg_readl(struct device *dev, enum isp_mem_resources isp_mmio_range,
		  u32 reg_offset)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	return __raw_readl(isp->mmio_base[isp_mmio_range] + reg_offset);
}
EXPORT_SYMBOL(isp_reg_readl);

/**
 * isp_reg_writel - Write value to an OMAP3 ISP register
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @reg_value: 32 bit value to write to the register.
 * @isp_mmio_range: Range to which the register offset refers to.
 * @reg_offset: Register offset to write into.
 **/
void isp_reg_writel(struct device *dev, u32 reg_value,
		    enum isp_mem_resources isp_mmio_range, u32 reg_offset)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	__raw_writel(reg_value, isp->mmio_base[isp_mmio_range] + reg_offset);
}
EXPORT_SYMBOL(isp_reg_writel);

/**
 * isp_reg_and - Do AND binary operation within an OMAP3 ISP register value
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @mmio_range: Range to which the register offset refers to.
 * @reg: Register offset to work on.
 * @and_bits: 32 bit value which would be 'ANDed' with current register value.
 **/
void isp_reg_and(struct device *dev, enum isp_mem_resources mmio_range, u32 reg,
		 u32 and_bits)
{
	u32 v = isp_reg_readl(dev, mmio_range, reg);

	isp_reg_writel(dev, v & and_bits, mmio_range, reg);
}
EXPORT_SYMBOL(isp_reg_and);

/**
 * isp_reg_or - Do OR binary operation within an OMAP3 ISP register value
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @mmio_range: Range to which the register offset refers to.
 * @reg: Register offset to work on.
 * @or_bits: 32 bit value which would be 'ORed' with current register value.
 **/
void isp_reg_or(struct device *dev, enum isp_mem_resources mmio_range, u32 reg,
		u32 or_bits)
{
	u32 v = isp_reg_readl(dev, mmio_range, reg);

	isp_reg_writel(dev, v | or_bits, mmio_range, reg);
}
EXPORT_SYMBOL(isp_reg_or);

/**
 * isp_reg_and_or - Do AND and OR binary ops within an OMAP3 ISP register value
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @mmio_range: Range to which the register offset refers to.
 * @reg: Register offset to work on.
 * @and_bits: 32 bit value which would be 'ANDed' with current register value.
 * @or_bits: 32 bit value which would be 'ORed' with current register value.
 *
 * The AND operation is done first, and then the OR operation. Mostly useful
 * when clearing a group of bits before setting a value.
 **/
void isp_reg_and_or(struct device *dev, enum isp_mem_resources mmio_range,
		    u32 reg, u32 and_bits, u32 or_bits)
{
	u32 v = isp_reg_readl(dev, mmio_range, reg);

	isp_reg_writel(dev, (v & and_bits) | or_bits, mmio_range, reg);
}
EXPORT_SYMBOL(isp_reg_and_or);

/**
 * find_vctrl - Return the index of the ctrl array of the requested ctrl ID.
 * @id: Requested control ID.
 *
 * Returns 0 if successful, -EINVAL if not found, or -EDOM if its out of
 * domain.
 **/
static int find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_control) - 1); i >= 0; i--)
		if (video_control[i].qc.id == id)
			break;

	if (i < 0)
		i = -EINVAL;

	return i;
}

/**
 * find_next_vctrl - Return next v4l2 ctrl ID available after the specified ID
 * @id: Reference V4L2 control ID.
 *
 * Returns 0 if successful, or -EINVAL if not found.
 **/
static int find_next_vctrl(int id)
{
	int i;
	u32 best = (u32)-1;

	for (i = 0; i < ARRAY_SIZE(video_control); i++) {
		if (video_control[i].qc.id > id &&
		    (best == (u32)-1 ||
		     video_control[i].qc.id <
		     video_control[best].qc.id)) {
			best = i;
		}
	}

	if (best == (u32)-1)
		return -EINVAL;

	return best;
}

/**
 * find_vmenu - Return index of the menu array of the requested ctrl option.
 * @id: Requested control ID.
 * @index: Requested menu option index.
 *
 * Returns 0 if successful, -EINVAL if not found, or -EDOM if its out of
 * domain.
 **/
static int find_vmenu(int id, int index)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(video_menu) - 1); i >= 0; i--) {
		if (video_menu[i].id != id || video_menu[i].index != index)
			continue;
		return i;
	}

	return -EINVAL;
}

/**
 * isp_release_resources - Free all currently requested ISP submodules.
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
static void isp_release_resources(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	if (isp->pipeline.modules & OMAP_ISP_CCDC)
		ispccdc_free(&isp->isp_ccdc);

	if (isp->pipeline.modules & OMAP_ISP_PREVIEW)
		isppreview_free(&isp->isp_prev);

	if (isp->pipeline.modules & OMAP_ISP_RESIZER)
		ispresizer_free(&isp->isp_res);
	return;
}

/**
 * isp_wait - Wait for idle or busy state transition with a time limit
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @busy: Function pointer which determines if submodule is busy.
 * @wait_for_busy: If 0, waits for idle state, if 1, waits for busy state.
 * @max_wait: Max retry count in us for wait for idle/busy transition.
 * @priv: Function parameter to send to busy check function.
 **/
static int isp_wait(struct device *dev, int (*busy)(void *), int wait_for_busy,
		    int max_wait, void *priv)
{
	int wait = 0;

	if (max_wait == 0)
		max_wait = 10000; /* 10 ms */

	while ((wait_for_busy && !busy(priv))
	       || (!wait_for_busy && busy(priv))) {
		rmb();
		udelay(1);
		wait++;
		if (wait > max_wait) {
			dev_alert(dev, "%s: wait is too much\n", __func__);
			return -EBUSY;
		}
	}
	DPRINTK_ISPCTRL(KERN_ALERT "%s: wait %d\n", __func__, wait);

	return 0;
}

/**
 * ispccdc_sbl_wait_idle - Wrapper to wait for ccdc sbl status bits to be idle.
 * @isp_ccdc: Pointer to ISP CCDC device.
 * @max_wait: Max retry count for wait for idle transition of the CCDC SBL bits
 **/
static int ispccdc_sbl_wait_idle(struct isp_ccdc_device *isp_ccdc, int max_wait)
{
	return isp_wait(isp_ccdc->dev, ispccdc_sbl_busy, 0, max_wait, isp_ccdc);
}

/**
 * isp_enable_interrupts - Enable ISP interrupts.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @is_raw: Determines if current pipeline ends in CCDC (!0) or Resizer (0)
 **/
static void isp_enable_interrupts(struct device *dev, int is_raw)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	u32 irq0enable;

	irq0enable = IRQ0ENABLE_CCDC_LSC_PREF_ERR_IRQ
		| IRQ0ENABLE_HS_VS_IRQ
		| IRQ0ENABLE_CCDC_VD0_IRQ | IRQ0ENABLE_CCDC_VD1_IRQ
		| IRQ0ENABLE_CSIA_IRQ
		| IRQ0ENABLE_CSIB_IRQ
		| IRQ0ENABLE_H3A_AWB_DONE_IRQ | IRQ0ENABLE_H3A_AF_DONE_IRQ
		| isp->interrupts;

	if (!is_raw)
		irq0enable |= IRQ0ENABLE_PRV_DONE_IRQ | IRQ0ENABLE_RSZ_DONE_IRQ;

	isp_reg_writel(dev, -1, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
	isp_reg_writel(dev, irq0enable, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE);

	return;
}

/**
 * isp_disable_interrupts - Disable all ISP interrupts.
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
static void isp_disable_interrupts(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	if (isp->bt656ifen == 0)
		isp_reg_writel(dev, 0, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE);
	else
		isp_reg_writel(dev, 0 | IRQ0ENABLE_RSZ_DONE_IRQ,
				OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE);
}

/**
 * isp_set_callback - Set an external callback for an ISP interrupt.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @type: Type of the event for which callback is requested.
 * @callback: Method to be called as callback in the ISR context.
 * @arg1: First argument to be passed when callback is called in ISR.
 * @arg2: Second argument to be passed when callback is called in ISR.
 *
 * This function sets a callback function for a done event in the ISP
 * module, and enables the corresponding interrupt.
 **/
int isp_set_callback(struct device *dev, enum isp_callback_type type,
		     isp_callback_t callback, isp_vbq_callback_ptr arg1,
		     void *arg2)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	unsigned long irqflags = 0;

	if (callback == NULL) {
		DPRINTK_ISPCTRL("ISP_ERR : Null Callback\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&isp->lock, irqflags);
	isp->irq.isp_callbk[type] = callback;
	isp->irq.isp_callbk_arg1[type] = arg1;
	isp->irq.isp_callbk_arg2[type] = arg2;
	spin_unlock_irqrestore(&isp->lock, irqflags);

	switch (type) {
	case CBK_HIST_DONE:
		isp->interrupts |= IRQ0ENABLE_HIST_DONE_IRQ;
		if (isp->running != ISP_RUNNING)
			break;
		isp_reg_writel(dev, IRQ0ENABLE_HIST_DONE_IRQ,
			       OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
		isp_reg_or(dev, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			   IRQ0ENABLE_HIST_DONE_IRQ);
		break;
	case CBK_PREV_DONE:
		isp_reg_writel(dev, IRQ0ENABLE_PRV_DONE_IRQ,
			       OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
		isp_reg_or(dev, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			   IRQ0ENABLE_PRV_DONE_IRQ);
		break;
	case CBK_RESZ_DONE:
		isp_reg_writel(dev, IRQ0ENABLE_RSZ_DONE_IRQ,
			       OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
		isp_reg_or(dev, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			   IRQ0ENABLE_RSZ_DONE_IRQ);
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(isp_set_callback);

/**
 * isp_unset_callback - Clears the callback for the ISP module done events.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @type: Type of the event for which callback to be cleared.
 *
 * This function clears a callback function for a done event in the ISP
 * module, and disables the corresponding interrupt.
 **/
int isp_unset_callback(struct device *dev, enum isp_callback_type type)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	unsigned long irqflags = 0;

	spin_lock_irqsave(&isp->lock, irqflags);
	isp->irq.isp_callbk[type] = NULL;
	isp->irq.isp_callbk_arg1[type] = NULL;
	isp->irq.isp_callbk_arg2[type] = NULL;
	spin_unlock_irqrestore(&isp->lock, irqflags);

	switch (type) {
	case CBK_HIST_DONE:
		isp->interrupts &= ~IRQ0ENABLE_HIST_DONE_IRQ;
		if (isp->running != ISP_RUNNING)
			break;
		isp_reg_and(dev, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			    ~IRQ0ENABLE_HIST_DONE_IRQ);
		break;
	case CBK_PREV_DONE:
		isp_reg_and(dev, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			    ~IRQ0ENABLE_PRV_DONE_IRQ);
		break;
	case CBK_RESZ_DONE:
		isp_reg_and(dev, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0ENABLE,
			    ~IRQ0ENABLE_RSZ_DONE_IRQ);
		break;
	default:
		break;
	}

	return 0;
}
EXPORT_SYMBOL(isp_unset_callback);

/**
 * isp_set_xclk - Configures the specified cam_xclk to the desired frequency.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @xclk: Desired frequency of the clock in Hz.
 * @xclksel: XCLK to configure (0 = A, 1 = B).
 *
 * Configures the specified MCLK divisor in the ISP timing control register
 * (TCTRL_CTRL) to generate the desired xclk clock value.
 *
 * Divisor = CM_CAM_MCLK_HZ / xclk
 *
 * Returns the final frequency that is actually being generated
 **/
u32 isp_set_xclk(struct device *dev, u32 xclk, u8 xclksel)
{
	u32 divisor;
	u32 currentxclk;

	if (xclk >= CM_CAM_MCLK_HZ) {
		divisor = ISPTCTRL_CTRL_DIV_BYPASS;
		currentxclk = CM_CAM_MCLK_HZ;
	} else if (xclk >= 2) {
		divisor = CM_CAM_MCLK_HZ / xclk;
		if (divisor >= ISPTCTRL_CTRL_DIV_BYPASS)
			divisor = ISPTCTRL_CTRL_DIV_BYPASS - 1;
		currentxclk = CM_CAM_MCLK_HZ / divisor;
	} else {
		divisor = xclk;
		currentxclk = 0;
	}

	switch (xclksel) {
	case 0:
		isp_reg_and_or(dev, OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_CTRL,
			       ~ISPTCTRL_CTRL_DIVA_MASK,
			       divisor << ISPTCTRL_CTRL_DIVA_SHIFT);
		DPRINTK_ISPCTRL("isp_set_xclk(): cam_xclka set to %d Hz\n",
				currentxclk);
		break;
	case 1:
		isp_reg_and_or(dev, OMAP3_ISP_IOMEM_MAIN, ISP_TCTRL_CTRL,
			       ~ISPTCTRL_CTRL_DIVB_MASK,
			       divisor << ISPTCTRL_CTRL_DIVB_SHIFT);
		DPRINTK_ISPCTRL("isp_set_xclk(): cam_xclkb set to %d Hz\n",
				currentxclk);
		break;
	default:
		DPRINTK_ISPCTRL("ISP_ERR: isp_set_xclk(): Invalid requested "
				"xclk. Must be 0 (A) or 1 (B).\n");
		return -EINVAL;
	}

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
static void isp_power_settings(struct device *dev, int idle)
{
	if (idle) {
		isp_reg_writel(dev, ISP_SYSCONFIG_AUTOIDLE |
			       (ISP_SYSCONFIG_MIDLEMODE_SMARTSTANDBY <<
				ISP_SYSCONFIG_MIDLEMODE_SHIFT),
			       OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG);
		if (!cpu_is_omap3630() && cpu_is_omap34xx() &&
				omap_rev_is_1_0()) {
			isp_reg_writel(dev, ISPCSI1_AUTOIDLE |
				       (ISPCSI1_MIDLEMODE_SMARTSTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
				       OMAP3_ISP_IOMEM_CSI2A,
				       ISP_CSIA_SYSCONFIG);
			isp_reg_writel(dev, ISPCSI1_AUTOIDLE |
				       (ISPCSI1_MIDLEMODE_SMARTSTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
				       OMAP3_ISP_IOMEM_CCP2,
				       ISP_CSIB_SYSCONFIG);
		}
		isp_reg_writel(dev, ISPCTRL_SBL_AUTOIDLE, OMAP3_ISP_IOMEM_MAIN,
			       ISP_CTRL);

	} else {
		isp_reg_writel(dev, ISP_SYSCONFIG_AUTOIDLE |
			       (ISP_SYSCONFIG_MIDLEMODE_FORCESTANDBY <<
				ISP_SYSCONFIG_MIDLEMODE_SHIFT),
			       OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG);
		if (!cpu_is_omap3630() && cpu_is_omap34xx() &&
				omap_rev_is_1_0()) {
			isp_reg_writel(dev, ISPCSI1_AUTOIDLE |
				       (ISPCSI1_MIDLEMODE_FORCESTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
				       OMAP3_ISP_IOMEM_CSI2A,
				       ISP_CSIA_SYSCONFIG);

			isp_reg_writel(dev, ISPCSI1_AUTOIDLE |
				       (ISPCSI1_MIDLEMODE_FORCESTANDBY <<
					ISPCSI1_MIDLEMODE_SHIFT),
				       OMAP3_ISP_IOMEM_CCP2,
				       ISP_CSIB_SYSCONFIG);
		}

		isp_reg_writel(dev, ISPCTRL_SBL_AUTOIDLE, OMAP3_ISP_IOMEM_MAIN,
			       ISP_CTRL);
	}
}

#define BIT_SET(var, shift, mask, val)		\
	do {					\
		var = (var & ~(mask << shift))	\
			| (val << shift);	\
	} while (0)

/**
 * isp_csi_enable - Enable CSI1/CCP2 interface.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @enable: Enable flag.
 **/
static void isp_csi_enable(struct device *dev, u8 enable)
{
	isp_reg_and_or(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL,
		       ~(BIT(0) | BIT(4)),
		       enable ? (BIT(0) | BIT(4)) : 0);
}

/**
 * isp_init_csi - Initialize CSI1/CCP2 interface.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @config: Pointer to ISP interface config structure.
 *
 * This will analize the parameters passed by the interface config
 * structure, and configure the respective registers for proper CSI1/CCP2
 * config.
 *
 * Returns -EINVAL if wrong format, -EIO if strobe is choosen in CSI1 mode, or
 * 0 on success.
 **/
static int isp_init_csi(struct device *dev, struct isp_interface_config *config)
{
	u32 i = 0, val, reg;
	int format;

	switch (config->u.csi.format) {
	case V4L2_PIX_FMT_SGRBG10:
		format = 0x16;		/* RAW10+VP */
		break;
	case V4L2_PIX_FMT_SGRBG10DPCM8:
		format = 0x12;		/* RAW8+DPCM10+VP */
		break;
	default:
		dev_err(dev, "isp_init_csi: bad csi format\n");
		return -EINVAL;
	}

	/* Reset the CSI and wait for reset to complete */
	isp_reg_writel(dev, isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2,
		       ISPCSI1_SYSCONFIG) | BIT(1),
		       OMAP3_ISP_IOMEM_CCP2, ISPCSI1_SYSCONFIG);
	while (!(isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_SYSSTATUS) &
		 BIT(0))) {
		udelay(10);
		if (i++ > 10)
			break;
	}
	if (!(isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_SYSSTATUS) &
	      BIT(0))) {
		dev_warn(dev,
		       "omap3_isp: timeout waiting for csi reset\n");
	}

	/* ISPCSI1_CTRL */
	val = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL);
	val &= ~BIT(11);	/* Enable VP only off ->
				   extract embedded data to interconnect */
	BIT_SET(val, 8, 0x3, config->u.csi.vpclk);	/* Video port clock */
/*	val |= BIT(3);	*/	/* Wait for FEC before disabling interface */
	val |= BIT(2);		/* I/O cell output is parallel
				   (no effect, but errata says should be enabled
				   for class 1/2) */
	val |= BIT(12);		/* VP clock polarity to falling edge
				   (needed or bad picture!) */

	/* Data/strobe physical layer */
	BIT_SET(val, 1, 1, config->u.csi.signalling);
	BIT_SET(val, 10, 1, config->u.csi.strobe_clock_inv);
	val |= BIT(4);		/* Magic bit to enable CSI1 and strobe mode */
	isp_reg_writel(dev, val, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_CTRL);

	/* ISPCSI1_LCx_CTRL logical channel #0 */
	reg = ISPCSI1_LCx_CTRL(0);	/* reg = ISPCSI1_CTRL1; */
	val = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, reg);
	/* Format = RAW10+VP or RAW8+DPCM10+VP*/
	BIT_SET(val, 3, 0x1f, format);
	/* Enable setting of frame regions of interest */
	BIT_SET(val, 1, 1, 1);
	BIT_SET(val, 2, 1, config->u.csi.crc);
	isp_reg_writel(dev, val, OMAP3_ISP_IOMEM_CCP2, reg);

	/* ISPCSI1_DAT_START for logical channel #0 */
	reg = ISPCSI1_LCx_DAT_START(0);		/* reg = ISPCSI1_DAT_START; */
	val = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, reg);
	BIT_SET(val, 16, 0xfff, config->u.csi.data_start);
	isp_reg_writel(dev, val, OMAP3_ISP_IOMEM_CCP2, reg);

	/* ISPCSI1_DAT_SIZE for logical channel #0 */
	reg = ISPCSI1_LCx_DAT_SIZE(0);		/* reg = ISPCSI1_DAT_SIZE; */
	val = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2, reg);
	BIT_SET(val, 16, 0xfff, config->u.csi.data_size);
	isp_reg_writel(dev, val, OMAP3_ISP_IOMEM_CCP2, reg);

	/* Clear status bits for logical channel #0 */
	val = ISPCSI1_LC01_IRQSTATUS_LC0_FIFO_OVF_IRQ |
	      ISPCSI1_LC01_IRQSTATUS_LC0_CRC_IRQ |
	      ISPCSI1_LC01_IRQSTATUS_LC0_FSP_IRQ |
	      ISPCSI1_LC01_IRQSTATUS_LC0_FW_IRQ |
	      ISPCSI1_LC01_IRQSTATUS_LC0_FSC_IRQ |
	      ISPCSI1_LC01_IRQSTATUS_LC0_SSC_IRQ;

	/* Clear IRQ status bits for logical channel #0 */
	isp_reg_writel(dev, val, OMAP3_ISP_IOMEM_CCP2,
		       ISPCSI1_LC01_IRQSTATUS);

	/* Enable IRQs for logical channel #0 */
	isp_reg_or(dev, OMAP3_ISP_IOMEM_CCP2, ISPCSI1_LC01_IRQENABLE, val);

	/* Enable CSI1 */
	isp_csi_enable(dev, 1);

	if (!(isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2,
			    ISPCSI1_CTRL) & BIT(4))) {
		dev_warn(dev, "OMAP3 CSI1 bus not available\n");
		if (config->u.csi.signalling) {
			/* Strobe mode requires CCP2 */
			return -EIO;
		}
	}

	return 0;
}

/**
 * isp_configure_interface - Configures ISP Control I/F related parameters.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @config: Pointer to structure containing the desired configuration for the
 *          ISP.
 *
 * Configures ISP control register (ISP_CTRL) with the values specified inside
 * the config structure. Controls:
 * - Selection of parallel or serial input to the preview hardware.
 * - Data lane shifter.
 * - Pixel clock polarity.
 * - 8 to 16-bit bridge at the input of CCDC module.
 * - HS or VS synchronization signal detection
 *
 * Returns 0 on success, otherwise, will return other negative error value.
 **/
int isp_configure_interface(struct device *dev,
			    struct isp_interface_config *config)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	u32 ispctrl_val = isp_reg_readl(dev, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL);
	u32 fmtcfg;
	int r;

	isp->config = config;

	ispctrl_val &= ISPCTRL_SHIFT_MASK;
	ispctrl_val |= config->dataline_shift << ISPCTRL_SHIFT_SHIFT;
	ispctrl_val &= ~ISPCTRL_PAR_CLK_POL_INV;

	ispctrl_val &= ISPCTRL_PAR_SER_CLK_SEL_MASK;

	isp_buf_init(dev);

	switch (config->ccdc_par_ser) {
	case ISP_PARLL:
	case ISP_PARLL_YUV_BT:
		ispctrl_val |= ISPCTRL_PAR_SER_CLK_SEL_PARALLEL;
		ispctrl_val |= config->u.par.par_clk_pol
			<< ISPCTRL_PAR_CLK_POL_SHIFT;
		ispctrl_val &= ~ISPCTRL_PAR_BRIDGE_BENDIAN;
		ispctrl_val |= config->u.par.par_bridge
			<< ISPCTRL_PAR_BRIDGE_SHIFT;
		if (config->ccdc_par_ser == ISP_PARLL_YUV_BT)
			isp->bt656ifen = 1;
		break;
	case ISP_CSIA:
		ispctrl_val |= ISPCTRL_PAR_SER_CLK_SEL_CSIA;
		ispctrl_val &= ~ISPCTRL_PAR_BRIDGE_BENDIAN;

		if (config->u.csi.crc)
			isp_csi2_ctrl_config_ecc_enable(true);

		isp_csi2_ctrl_config_vp_out_ctrl(config->u.csi.vpclk);
		isp_csi2_ctrl_config_vp_only_enable(true);
		isp_csi2_ctrl_config_vp_clk_enable(true);
		isp_csi2_ctrl_update(false);

		isp_csi2_ctx_config_format(0, config->u.csi.format);
		isp_csi2_ctx_update(0, false);

		isp_csi2_irq_complexio1_set(1);
		isp_csi2_irq_status_set(1);

		isp_csi2_enable(1);
		mdelay(3);
		break;
	case ISP_CSIB:
		ispctrl_val |= ISPCTRL_PAR_SER_CLK_SEL_CSIB;
		r = isp_init_csi(dev, config);
		if (r)
			return r;
		break;
	case ISP_NONE:
		return 0;
	default:
		return -EINVAL;
	}

	ispctrl_val &= ~ISPCTRL_SYNC_DETECT_VSRISE;
	ispctrl_val |= config->hsvs_syncdetect;

	isp_reg_writel(dev, ispctrl_val, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL);

	/* Set sensor specific fields in CCDC and Previewer module. */
	ispccdc_set_wenlog(&isp->isp_ccdc, config->wenlog);

	/* FIXME: this should be set in ispccdc_config_vp() */
	fmtcfg = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMTCFG);
	fmtcfg &= ISPCCDC_FMTCFG_VPIF_FRQ_MASK;
	if (config->pixelclk) {
		unsigned long l3_ick = clk_get_rate(isp->l3_ick);
		unsigned long div = l3_ick / config->pixelclk;
		if (div < 2)
			div = 2;
		if (div > 6)
			div = 6;
		fmtcfg |= (div - 2) << ISPCCDC_FMTCFG_VPIF_FRQ_SHIFT;
	}
	isp_reg_writel(dev, fmtcfg, OMAP3_ISP_IOMEM_CCDC, ISPCCDC_FMTCFG);

	return 0;
}
EXPORT_SYMBOL(isp_configure_interface);

static int isp_buf_process(struct device *dev, struct isp_bufs *bufs);

/**
 * omap34xx_isp_isr - Interrupt Service Routine for Camera ISP module.
 * @irq: Not used currently.
 * @_pdev: Pointer to the platform device associated with the OMAP3 ISP.
 *
 * Returns IRQ_HANDLED when IRQ was correctly handled, or IRQ_NONE when the
 * IRQ wasn't handled.
 **/
static irqreturn_t omap34xx_isp_isr(int irq, void *_pdev)
{
	struct device *dev = &((struct platform_device *)_pdev)->dev;
	struct isp_device *isp = dev_get_drvdata(dev);
	struct isp_irq *irqdis = &isp->irq;
	struct isp_bufs *bufs = &isp->bufs;
	unsigned long flags;
	u32 irqstatus = 0;
	u32 sbl_pcr;
	unsigned long irqflags = 0;
	int wait_hs_vs = 0;
	u8 fld_stat;

	if (isp->running == ISP_STOPPED) {
		dev_err(dev, "ouch %8.8x!\n",isp_reg_readl(dev, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS));
		return IRQ_NONE;
	}

	irqstatus = isp_reg_readl(dev, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
	isp_reg_writel(dev, irqstatus, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);

	if (isp->running == ISP_STOPPING)
		return IRQ_HANDLED;

	spin_lock_irqsave(&bufs->lock, flags);
	wait_hs_vs = bufs->wait_hs_vs;
	if (irqstatus & HS_VS) {
		if (bufs->wait_hs_vs) {
			bufs->wait_hs_vs--;
		} else {
			if (isp->pipeline.pix.field == V4L2_FIELD_INTERLACED) {
				fld_stat = (isp_reg_readl(dev,
					OMAP3_ISP_IOMEM_CCDC,
					ISPCCDC_SYN_MODE) &
					ISPCCDC_SYN_MODE_FLDSTAT) ? 1 :	0;
				isp->current_field = fld_stat;
			}
		}
	}
	spin_unlock_irqrestore(&bufs->lock, flags);

	spin_lock_irqsave(&isp->lock, irqflags);

	if (irqstatus & RESZ_DONE) {
		if (irqdis->isp_callbk[CBK_RESZ_DONE])
			irqdis->isp_callbk[CBK_RESZ_DONE](
				RESZ_DONE,
				irqdis->isp_callbk_arg1[CBK_RESZ_DONE],
				irqdis->isp_callbk_arg2[CBK_RESZ_DONE]);
		else if (!RAW_CAPTURE(isp)) {
			ispresizer_config_shadow_registers(&isp->isp_res);
			isp_buf_process(dev, bufs);
		}
	}
	/*
	 * We need to wait for the first HS_VS interrupt from CCDC.
	 * Otherwise our frame (and everything else) might be bad.
	 */
	switch (wait_hs_vs) {
	case 1:
		/*
		 * Enable preview for the first time. We just have
		 * missed the start-of-frame so we can do it now.
		 */
		if (irqstatus & HS_VS && !RAW_CAPTURE(isp))
			isppreview_enable(&isp->isp_prev);
	default:
		goto out_ignore_buff;
	case 0:
		break;
	}

	if (irqstatus & CCDC_VD0) {
		if (isp->pipeline.pix.field == V4L2_FIELD_INTERLACED) {
			/* Skip even fields, and process only odd fields */
			if (isp->current_field != 0)
				if (RAW_CAPTURE(isp))
					isp_buf_process(dev, bufs);
		}
		if (!ispccdc_busy(&isp->isp_ccdc))
			ispccdc_config_shadow_registers(&isp->isp_ccdc);
	}

	if (irqstatus & PREV_DONE) {
		if (irqdis->isp_callbk[CBK_PREV_DONE])
			irqdis->isp_callbk[CBK_PREV_DONE](
				PREV_DONE,
				irqdis->isp_callbk_arg1[CBK_PREV_DONE],
				irqdis->isp_callbk_arg2[CBK_PREV_DONE]);
		else if (!RAW_CAPTURE(isp)) {
			if (ispresizer_busy(&isp->isp_res)) {
				ISP_BUF_DONE(bufs)->vb_state =
					VIDEOBUF_ERROR;
				dev_err(dev, "%s: resizer busy!\n", __func__);
			} else {
				ispresizer_enable(&isp->isp_res, 1);
			}
			isppreview_config_shadow_registers(&isp->isp_prev);
			isppreview_enable(&isp->isp_prev);
		}
	}

	if (irqstatus & H3A_AWB_DONE)
		isph3a_aewb_isr(&isp->isp_h3a);

	if (irqstatus & HIST_DONE) {
		if (irqdis->isp_callbk[CBK_HIST_DONE])
			irqdis->isp_callbk[CBK_HIST_DONE](
				HIST_DONE,
				irqdis->isp_callbk_arg1[CBK_HIST_DONE],
				irqdis->isp_callbk_arg2[CBK_HIST_DONE]);
	}

	if (irqstatus & H3A_AF_DONE)
		isp_af_isr(&isp->isp_af);

	/* Handle shared buffer logic overflows for video buffers. */
	/* ISPSBL_PCR_CCDCPRV_2_RSZ_OVF can be safely ignored. */
	sbl_pcr = isp_reg_readl(dev, OMAP3_ISP_IOMEM_SBL, ISPSBL_PCR) &
		~ISPSBL_PCR_CCDCPRV_2_RSZ_OVF;
	isp_reg_writel(dev, sbl_pcr, OMAP3_ISP_IOMEM_SBL, ISPSBL_PCR);
	if (sbl_pcr & (ISPSBL_PCR_RSZ1_WBL_OVF
		       | ISPSBL_PCR_RSZ2_WBL_OVF
		       | ISPSBL_PCR_RSZ3_WBL_OVF
		       | ISPSBL_PCR_RSZ4_WBL_OVF
		       | ISPSBL_PCR_PRV_WBL_OVF
		       | ISPSBL_PCR_CCDC_WBL_OVF
		       | ISPSBL_PCR_CSIA_WBL_OVF
		       | ISPSBL_PCR_CSIB_WBL_OVF)) {
		struct isp_buf *buf = ISP_BUF_DONE(bufs);
		buf->vb_state = VIDEOBUF_ERROR;
		dev_info(dev, "%s: sbl overflow, sbl_pcr = %8.8x\n",
		       __func__, sbl_pcr);
	}

out_ignore_buff:
	if (irqstatus & LSC_PRE_ERR) {
		struct isp_buf *buf = ISP_BUF_DONE(bufs);
		/* Mark buffer faulty. */
		buf->vb_state = VIDEOBUF_ERROR;
		ispccdc_lsc_error_handler(&isp->isp_ccdc);
		dev_err(dev, "%s: lsc prefetch error\n", __func__);
	}

	if (irqstatus & CSIA) {
		struct isp_buf *buf = ISP_BUF_DONE(bufs);
		int ret = isp_csi2_isr();
		if (ret)
			buf->vb_state = VIDEOBUF_ERROR;
	}

	if (irqstatus & IRQ0STATUS_CSIB_IRQ) {
		struct isp_buf *buf = ISP_BUF_DONE(bufs);
		u32 ispcsi1_irqstatus;

		ispcsi1_irqstatus = isp_reg_readl(dev, OMAP3_ISP_IOMEM_CCP2,
						  ISPCSI1_LC01_IRQSTATUS);
		isp_reg_writel(dev, ispcsi1_irqstatus, OMAP3_ISP_IOMEM_CCP2,
			       ISPCSI1_LC01_IRQSTATUS);
		buf->vb_state = VIDEOBUF_ERROR;
		dev_err(dev, "CCP2 err:%x\n", ispcsi1_irqstatus);
	}

	if (irqdis->isp_callbk[CBK_CATCHALL]) {
		irqdis->isp_callbk[CBK_CATCHALL](
			irqstatus,
			irqdis->isp_callbk_arg1[CBK_CATCHALL],
			irqdis->isp_callbk_arg2[CBK_CATCHALL]);
	}

	spin_unlock_irqrestore(&isp->lock, irqflags);

	isp_flush(dev);

#if 1
	{
		static const struct {
			int num;
			char *name;
		} bits[] = {
			{ 31, "HS_VS_IRQ" },
			{ 30, "SEC_ERR_IRQ" },
			{ 29, "OCP_ERR_IRQ" },
			{ 28, "MMU_ERR_IRQ" },
			{ 27, "res27" },
			{ 26, "res26" },
			{ 25, "OVF_IRQ" },
			{ 24, "RSZ_DONE_IRQ" },
			{ 23, "res23" },
			{ 22, "res22" },
			{ 21, "CBUFF_IRQ" },
			{ 20, "PRV_DONE_IRQ" },
			{ 19, "CCDC_LSC_PREFETCH_ERROR" },
			{ 18, "CCDC_LSC_PREFETCH_COMPLETED" },
			{ 17, "CCDC_LSC_DONE" },
			{ 16, "HIST_DONE_IRQ" },
			{ 15, "res15" },
			{ 14, "res14" },
			{ 13, "H3A_AWB_DONE_IRQ" },
			{ 12, "H3A_AF_DONE_IRQ" },
			{ 11, "CCDC_ERR_IRQ" },
			{ 10, "CCDC_VD2_IRQ" },
			{  9, "CCDC_VD1_IRQ" },
			{  8, "CCDC_VD0_IRQ" },
			{  7, "res7" },
			{  6, "res6" },
			{  5, "res5" },
			{  4, "CSIB_IRQ" },
			{  3, "CSIB_LCM_IRQ" },
			{  2, "res2" },
			{  1, "res1" },
			{  0, "CSIA_IRQ" },
		};
		int i;
		for (i = 0; i < ARRAY_SIZE(bits); i++) {
			if ((1 << bits[i].num) & irqstatus)
				DPRINTK_ISPCTRL("%s ", bits[i].name);
		}
		DPRINTK_ISPCTRL("\n");
	}
#endif

	return IRQ_HANDLED;
}

/* Device name, needed for resource tracking layer */
struct device_driver camera_drv = {
	.name = "camera"
};

struct device camera_dev = {
	.driver = &camera_drv,
};

/**
 * isp_tmp_buf_free - Free buffer for CCDC->PRV->RSZ datapath workaround.
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
static void isp_tmp_buf_free(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	if (isp->tmp_buf) {
		iommu_vfree(isp->iommu, isp->tmp_buf);
		isp->tmp_buf = 0;
		isp->tmp_buf_size = 0;
	}
}

/**
 * isp_tmp_buf_alloc - Allocate buffer for CCDC->PRV->RSZ datapath workaround.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @size: Byte size of the buffer to allocate
 *
 * Returns 0 if successful, or -ENOMEM if there's no available memory.
 **/
static u32 isp_tmp_buf_alloc(struct device *dev, size_t size)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	u32 da;

	isp_tmp_buf_free(dev);

	dev_dbg(dev, "%s: allocating %d bytes\n", __func__, size);

	da = iommu_vmalloc(isp->iommu, 0, size, IOMMU_FLAG);
	if (IS_ERR_VALUE(da)) {
		dev_err(dev, "iommu_vmap mapping failed ");
		return -ENOMEM;
	}
	isp->tmp_buf = da;
	isp->tmp_buf_size = size;

	isppreview_set_outaddr(&isp->isp_prev, isp->tmp_buf);
	ispresizer_set_inaddr(&isp->isp_res, isp->tmp_buf);

	return 0;
}

/**
 * isp_start - Set ISP in running state
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
void isp_start(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	isp->running = ISP_RUNNING;

	return;
}
EXPORT_SYMBOL(isp_start);

#define ISP_STATISTICS_BUSY			\
	()
#define ISP_STOP_TIMEOUT	msecs_to_jiffies(1000)

/**
 * __isp_disable_modules - Disable ISP submodules with a timeout to be idle.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @suspend: If 0, disable modules; if 1, send modules to suspend state.
 *
 * Returns 0 if stop/suspend left in idle state all the submodules properly,
 * or returns 1 if a general Reset is required to stop/suspend the submodules.
 **/
static int __isp_disable_modules(struct device *dev, int suspend)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	unsigned long timeout = jiffies + ISP_STOP_TIMEOUT;
	int reset = 0;

	/*
	 * We need to stop all the modules after CCDC first or they'll
	 * never stop since they may not get a full frame from CCDC.
	 */
	if (suspend) {
		isp_af_suspend(&isp->isp_af);
		isph3a_aewb_suspend(&isp->isp_h3a);
		isp_hist_suspend(&isp->isp_hist);
	} else {
		isp_af_enable(&isp->isp_af, 0);
		isph3a_aewb_enable(&isp->isp_h3a, 0);
		isp_hist_enable(&isp->isp_hist, 0);
	}
	ispresizer_enable(&isp->isp_res, 0);

	timeout = jiffies + ISP_STOP_TIMEOUT;
	while (isp_af_busy(&isp->isp_af)
	       || isph3a_aewb_busy(&isp->isp_h3a)
	       || isp_hist_busy(&isp->isp_hist)
	       || isppreview_busy(&isp->isp_prev)
	       || ispresizer_busy(&isp->isp_res)) {
		if (time_after(jiffies, timeout)) {
			dev_err(dev, "%s: can't stop non-ccdc modules\n",
			       __func__);
			reset = 1;
			break;
		}
		msleep(1);
	}

	/* Let's stop CCDC now. */
	ispccdc_enable(&isp->isp_ccdc, 0);

	timeout = jiffies + ISP_STOP_TIMEOUT;
	while (ispccdc_busy(&isp->isp_ccdc)) {
		if (time_after(jiffies, timeout)) {
			dev_err(dev, "%s: can't stop ccdc\n", __func__);
			reset = 1;
			break;
		}
		msleep(1);
	}

	isp_csi_enable(dev, 0);
	isp_csi2_enable(0);
	isp_buf_init(dev);

	return reset;
}

/**
 * isp_stop_modules - Stop ISP submodules.
 * @dev: Device pointer specific to the OMAP3 ISP.
 *
 * Returns 0 if stop left in idle state all the submodules properly,
 * or returns 1 if a general Reset is required to stop the submodules.
 **/
static int isp_stop_modules(struct device *dev)
{
	return __isp_disable_modules(dev, 0);
}

#ifdef CONFIG_PM
/**
 * isp_suspend_modules - Suspend ISP submodules.
 * @dev: Device pointer specific to the OMAP3 ISP.
 *
 * Returns 0 if suspend left in idle state all the submodules properly,
 * or returns 1 if a general Reset is required to suspend the submodules.
 **/
static int isp_suspend_modules(struct device *dev)
{
	return __isp_disable_modules(dev, 1);
}

/**
 * isp_resume_modules - Resume ISP submodules.
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
static void isp_resume_modules(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	isp_hist_resume(&isp->isp_hist);
	isph3a_aewb_resume(&isp->isp_h3a);
	isp_af_resume(&isp->isp_af);
}
#endif	/* CONFIG_PM */

/**
 * isp_reset - Reset ISP with a timeout wait for idle.
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
static void isp_reset(struct device *dev)
{
	unsigned long timeout = 0;

	isp_reg_writel(dev,
		       isp_reg_readl(dev, OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG)
		       | ISP_SYSCONFIG_SOFTRESET,
		       OMAP3_ISP_IOMEM_MAIN, ISP_SYSCONFIG);
	while (!(isp_reg_readl(dev, OMAP3_ISP_IOMEM_MAIN,
			       ISP_SYSSTATUS) & 0x1)) {
		if (timeout++ > 10000) {
			dev_alert(dev, "%s: cannot reset ISP\n", __func__);
			break;
		}
		udelay(1);
	}
}

/**
 * isp_stop - Stop ISP.
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
void isp_stop(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	int reset;

	isp->running = ISP_STOPPING;
	isp_disable_interrupts(dev);
	synchronize_irq(((struct isp_device *)dev_get_drvdata(dev))->irq_num);
	isp->running = ISP_STOPPED;
	reset = isp_stop_modules(dev);
	if (!reset)
		return;

	isp_save_ctx(dev);
	isp_reset(dev);
	isp_restore_ctx(dev);
}
EXPORT_SYMBOL(isp_stop);

/**
 * isp_set_buf - Program output buffer address based on current pipeline.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @buf: Pointer to ISP buffer structure.
 **/
static void isp_set_buf(struct device *dev, struct isp_buf *buf)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	if (isp->pipeline.modules & OMAP_ISP_RESIZER
	    && is_ispresizer_enabled())
		ispresizer_set_outaddr(&isp->isp_res, buf->isp_addr);
	else if (isp->pipeline.modules & OMAP_ISP_CCDC)
		ispccdc_set_outaddr(&isp->isp_ccdc, buf->isp_addr);

}

/**
 * isp_try_pipeline - Retrieve and simulate resulting internal ISP pipeline.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @pix_input: Pointer to pixel format to use as input in the ISP.
 * @pipe: Pointer to ISP pipeline structure to fill back.
 *
 * Returns the closest possible output size based on silicon limitations
 * detailed through the pipe structure.
 *
 * If the input can't be read, it'll return -EINVAL. Returns 0 on success.
 **/
static int isp_try_pipeline(struct device *dev,
			    struct v4l2_pix_format *pix_input,
			    struct isp_pipeline *pipe)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	struct v4l2_pix_format *pix_output = &pipe->pix;
	unsigned int wanted_width = pix_output->width;
	unsigned int wanted_height = pix_output->height;
	int ifmt;
	int rval;

	if ((pix_input->pixelformat == V4L2_PIX_FMT_SGRBG10 ||
	     pix_input->pixelformat == V4L2_PIX_FMT_SGRBG10DPCM8 ||
	     pix_input->pixelformat == V4L2_PIX_FMT_SRGGB10 ||
	     pix_input->pixelformat == V4L2_PIX_FMT_SBGGR10 ||
	     pix_input->pixelformat == V4L2_PIX_FMT_SGBRG10) &&
	    (pix_output->pixelformat == V4L2_PIX_FMT_YUYV ||
	     pix_output->pixelformat == V4L2_PIX_FMT_UYVY)) {
		pipe->modules = OMAP_ISP_CCDC | OMAP_ISP_PREVIEW
			| OMAP_ISP_RESIZER;
		if (pix_input->pixelformat == V4L2_PIX_FMT_SGRBG10 ||
		    pix_input->pixelformat == V4L2_PIX_FMT_SGRBG10DPCM8)
			pipe->ccdc_in = CCDC_RAW_GRBG;
		if (pix_input->pixelformat == V4L2_PIX_FMT_SRGGB10)
			pipe->ccdc_in = CCDC_RAW_RGGB;
		if (pix_input->pixelformat == V4L2_PIX_FMT_SBGGR10)
			pipe->ccdc_in = CCDC_RAW_BGGR;
		if (pix_input->pixelformat == V4L2_PIX_FMT_SGBRG10)
			pipe->ccdc_in = CCDC_RAW_GBRG;
		pipe->ccdc_out = CCDC_OTHERS_VP;
		pipe->prv_in = PRV_RAW_CCDC;
		pipe->prv_out = PREVIEW_MEM;
		pipe->rsz_in = RSZ_MEM_YUV;
	} else {
		pipe->modules = OMAP_ISP_CCDC;
		if (pix_input->pixelformat == V4L2_PIX_FMT_SGRBG10 ||
		    pix_input->pixelformat == V4L2_PIX_FMT_SGRBG10DPCM8 ||
		    pix_input->pixelformat == V4L2_PIX_FMT_SRGGB10 ||
		    pix_input->pixelformat == V4L2_PIX_FMT_SBGGR10 ||
		    pix_input->pixelformat == V4L2_PIX_FMT_SGBRG10) {
			pipe->ccdc_out = CCDC_OTHERS_VP_MEM;
			if (pix_input->pixelformat == V4L2_PIX_FMT_SGRBG10 ||
			    pix_input->pixelformat == V4L2_PIX_FMT_SGRBG10DPCM8)
				pipe->ccdc_in = CCDC_RAW_GRBG;
			if (pix_input->pixelformat == V4L2_PIX_FMT_SRGGB10)
				pipe->ccdc_in = CCDC_RAW_RGGB;
			if (pix_input->pixelformat == V4L2_PIX_FMT_SBGGR10)
				pipe->ccdc_in = CCDC_RAW_BGGR;
			if (pix_input->pixelformat == V4L2_PIX_FMT_SGBRG10)
				pipe->ccdc_in = CCDC_RAW_GBRG;
		} else if (pix_input->pixelformat == V4L2_PIX_FMT_YUYV ||
			   pix_input->pixelformat == V4L2_PIX_FMT_UYVY) {
			if (isp->bt656ifen)
				pipe->ccdc_in = CCDC_YUV_BT;
			else
				pipe->ccdc_in = CCDC_YUV_SYNC;
			pipe->ccdc_out = CCDC_OTHERS_MEM;
		} else
			return -EINVAL;
	}

	if (pipe->modules & OMAP_ISP_CCDC) {
		pipe->ccdc_in_w = pix_input->width;
		pipe->ccdc_in_h = pix_input->height;
		rval = ispccdc_try_pipeline(&isp->isp_ccdc, pipe);
		if (rval) {
			dev_err(dev, "the dimensions %dx%d are not"
			       " supported\n", pix_input->width,
			       pix_input->height);
			return rval;
		}
		pix_output->width = pipe->ccdc_out_w_img;
		pix_output->height = pipe->ccdc_out_h;
		pix_output->bytesperline =
			pipe->ccdc_out_w * ISP_BYTES_PER_PIXEL;
	}

	if (pipe->modules & OMAP_ISP_PREVIEW) {
		rval = isppreview_try_pipeline(&isp->isp_prev, pipe);
		if (rval) {
			dev_err(dev, "the dimensions %dx%d are not"
			       " supported\n", pix_input->width,
			       pix_input->height);
			return rval;
		}
		pix_output->width = pipe->prv_out_w;
		pix_output->height = pipe->prv_out_h;
	}

	if (pipe->modules & OMAP_ISP_RESIZER) {
		pipe->rsz_out_w = wanted_width;
		pipe->rsz_out_h = wanted_height;

		pipe->rsz_crop.left = pipe->rsz_crop.top = 0;
		pipe->rsz_crop.width = pipe->prv_out_w_img;
		pipe->rsz_crop.height = pipe->prv_out_h_img;

		rval = ispresizer_try_pipeline(&isp->isp_res, pipe);
		if (rval) {
			dev_err(dev, "The dimensions %dx%d are not"
			       " supported\n", pix_input->width,
			       pix_input->height);
			return rval;
		}

		pix_output->width = pipe->rsz_out_w;
		pix_output->height = pipe->rsz_out_h;
		pix_output->bytesperline =
			pipe->rsz_out_w * ISP_BYTES_PER_PIXEL;
	}

	if (isp->bt656ifen)
		pix_output->field = pix_input->field;
	else {
		pix_output->field = V4L2_FIELD_NONE;
		pix_output->sizeimage =
			PAGE_ALIGN(pix_output->bytesperline *
					pix_output->height);
	}
	pix_output->priv = 0;

	for (ifmt = 0; ifmt < NUM_ISP_CAPTURE_FORMATS; ifmt++) {
		if (pix_output->pixelformat == isp_formats[ifmt].pixelformat)
			break;
	}
	if (ifmt == NUM_ISP_CAPTURE_FORMATS)
		pix_output->pixelformat = V4L2_PIX_FMT_YUYV;

	switch (pix_output->pixelformat) {
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_UYVY:
		if (isp->bt656ifen)
			pix_output->colorspace = pix_input->colorspace;
		else
			pix_output->colorspace = V4L2_COLORSPACE_JPEG;
		break;
	default:
		pix_output->colorspace = V4L2_COLORSPACE_SRGB;
	}

	return 0;
}

/**
 * isp_s_pipeline - Configure internal ISP pipeline.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @pix_input: Pointer to pixel format to use as input in the ISP.
 * @pix_output: Pointer to pixel format to use as output in the ISP.
 *
 * Returns the closest possible output size based on silicon limitations.
 *
 * If the input can't be read, it'll return -EINVAL. Returns 0 on success.
 **/
static int isp_s_pipeline(struct device *dev,
			  struct v4l2_pix_format *pix_input,
			  struct v4l2_pix_format *pix_output)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	struct isp_pipeline pipe;
	int rval;

	isp_release_resources(dev);

	pipe.pix = *pix_output;

	rval = isp_try_pipeline(dev, pix_input, &pipe);
	if (rval)
		return rval;

	ispccdc_request(&isp->isp_ccdc);
	ispccdc_s_pipeline(&isp->isp_ccdc, &pipe);

	if (pix_input->pixelformat == V4L2_PIX_FMT_UYVY)
		ispccdc_config_y8pos(&isp->isp_ccdc, Y8POS_ODD);
	else if (pix_input->pixelformat == V4L2_PIX_FMT_YUYV)
		ispccdc_config_y8pos(&isp->isp_ccdc, Y8POS_EVEN);

	if (((pix_input->pixelformat == V4L2_PIX_FMT_UYVY) &&
			(pix_output->pixelformat == V4L2_PIX_FMT_UYVY))	||
			((pix_input->pixelformat == V4L2_PIX_FMT_YUYV) &&
			 (pix_output->pixelformat == V4L2_PIX_FMT_YUYV)))
		/* input and output formats are in same order */
		ispccdc_config_byteswap(&isp->isp_ccdc, 0);
	else if (((pix_input->pixelformat == V4L2_PIX_FMT_YUYV) &&
			(pix_output->pixelformat == V4L2_PIX_FMT_UYVY)) ||
			((pix_input->pixelformat == V4L2_PIX_FMT_UYVY) &&
			(pix_output->pixelformat == V4L2_PIX_FMT_YUYV)))
		/* input and output formats are in reverse order */
		ispccdc_config_byteswap(&isp->isp_ccdc, 1);
	/*
	 * Configure Pitch - This enables application to use a
	 * different pitch
	 * other than active pixels per line.
	 */
	if (isp->bt656ifen)
		ispccdc_config_outlineoffset(&isp->isp_ccdc,
				pipe.pix.bytesperline, 0, 0);
	if (pipe.modules & OMAP_ISP_PREVIEW) {
		isppreview_request(&isp->isp_prev);
		isppreview_s_pipeline(&isp->isp_prev, &pipe);
	}

	if (pipe.modules & OMAP_ISP_RESIZER) {
		ispresizer_request(&isp->isp_res);
		ispresizer_s_pipeline(&isp->isp_res, &pipe);
	}

	isp->pipeline = pipe;
	*pix_output = isp->pipeline.pix;

	return 0;
}

/**
 * isp_vbq_sync - Flush the entire cache
 * @vb: Videobuffer to sync. (Not used)
 * @when: (Not used)
 *
 * FIXME: This impacts the performance on the other systems when camera is
 * running, but seems to be needed to ensure coherency of DMA transfers
 * somehow. Investigation ongoing...
 **/
static int isp_vbq_sync(struct videobuf_buffer *vb, int when)
{
	flush_cache_all();

	return 0;
}

/**
 * isp_buf_init - Initialize the internal buffer queue handling.
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
static void isp_buf_init(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	struct isp_bufs *bufs = &isp->bufs;
	int sg;

	bufs->queue = 0;
	bufs->done = 0;
	bufs->wait_hs_vs = isp->config->wait_hs_vs;
	for (sg = 0; sg < NUM_BUFS; sg++) {
		if (bufs->buf[sg].vb) {
			isp_vbq_sync(bufs->buf[sg].vb, DMA_FROM_DEVICE);
			bufs->buf[sg].vb->state = VIDEOBUF_ERROR;
			bufs->buf[sg].complete(bufs->buf[sg].vb,
					       bufs->buf[sg].priv);
		}
		bufs->buf[sg].complete = NULL;
		bufs->buf[sg].vb = NULL;
		bufs->buf[sg].priv = NULL;
	}
}

/**
 * isp_buf_process - Do final handling when a buffer has been processed.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @bufs: Pointer to ISP buffer handling structure.
 *
 * Updates the pointers accordingly depending of the internal pipeline.
 **/
static int isp_buf_process(struct device *dev, struct isp_bufs *bufs)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	struct isp_buf *buf = NULL;
	unsigned long flags;
	int last;

	spin_lock_irqsave(&bufs->lock, flags);

	if (ISP_BUFS_IS_EMPTY(bufs))
		goto out;

	if (RAW_CAPTURE(isp) && ispccdc_sbl_wait_idle(&isp->isp_ccdc, 1000)) {
		dev_err(dev, "ccdc %d won't become idle!\n",
		       RAW_CAPTURE(isp));
		goto out;
	}

	/* We had at least one buffer in queue. */
	buf = ISP_BUF_DONE(bufs);
	last = ISP_BUFS_IS_LAST(bufs);

	if (!last) {
		/* Set new buffer address. */
		isp_set_buf(dev, ISP_BUF_NEXT_DONE(bufs));
	} else {
		/* Tell ISP not to write any of our buffers. */
		isp_disable_interrupts(dev);
		if (RAW_CAPTURE(isp))
			ispccdc_enable(&isp->isp_ccdc, 0);
		else if (isp->bt656ifen == 0)
			ispresizer_enable(&isp->isp_res, 0);
		/*
		 * We must wait for the HS_VS since before that the
		 * CCDC may trigger interrupts even if it's not
		 * receiving a frame.
		 */
		bufs->wait_hs_vs = isp->config->wait_hs_vs;
	}
	if ((RAW_CAPTURE(isp) && ispccdc_busy(&isp->isp_ccdc))
	    || (!RAW_CAPTURE(isp) && ispresizer_busy(&isp->isp_res))) {
		/*
		 * Next buffer available: for the transfer to succeed, the
		 * CCDC (RAW capture) or resizer (YUV capture) must be idle
		 * for the duration of transfer setup. Bad things happen
		 * otherwise!
		 *
		 * Next buffer not available: if we fail to stop the
		 * ISP the buffer is probably going to be bad.
		 */
		/* Mark this buffer faulty. */
		buf->vb_state = VIDEOBUF_ERROR;
		/* Mark next faulty, too, in case we have one. */
		if (!last) {
			ISP_BUF_NEXT_DONE(bufs)->vb_state = VIDEOBUF_ERROR;
			dev_alert(dev, "OUCH!!!\n");
		} else {
			dev_alert(dev, "Ouch!\n");
		}
	}

	/* Mark the current buffer as done. */
	ISP_BUF_MARK_DONE(bufs);

	DPRINTK_ISPCTRL(KERN_ALERT "%s: finish %d mmu %p\n", __func__,
			(bufs->done - 1 + NUM_BUFS) % NUM_BUFS,
			(bufs->buf+((bufs->done - 1 + NUM_BUFS)
				    % NUM_BUFS))->isp_addr);

out:
	spin_unlock_irqrestore(&bufs->lock, flags);

	if (buf && buf->vb) {
		/*
		 * We want to dequeue a buffer from the video buffer
		 * queue. Let's do it!
		 */
		isp_vbq_sync(buf->vb, DMA_FROM_DEVICE);
		buf->vb->state = buf->vb_state;
		buf->complete(buf->vb, buf->priv);
		buf->vb = NULL;
	}

	return 0;
}

/**
 * isp_buf_queue - Queue a buffer into the internal ISP queue list.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @vb: Pointer to video buffer to queue.
 * @complete: Pointer to function to call when buffer is completely processed.
 * @priv: Pointer to private paramemter to send to complete function.
 *
 * Always returns 0.
 **/
int isp_buf_queue(struct device *dev, struct videobuf_buffer *vb,
		  void (*complete)(struct videobuf_buffer *vb, void *priv),
		  void *priv)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	unsigned long flags;
	struct isp_buf *buf;
	struct videobuf_dmabuf *dma = videobuf_to_dma(vb);
	const struct scatterlist *sglist = dma->sglist;
	struct isp_bufs *bufs = &isp->bufs;
	int sglen = dma->sglen;

	if (isp->running != ISP_RUNNING) {
		vb->state = VIDEOBUF_ERROR;
		complete(vb, priv);

		return 0;
	}

	BUG_ON(sglen < 0 || !sglist);

	isp_vbq_sync(vb, DMA_TO_DEVICE);

	spin_lock_irqsave(&bufs->lock, flags);

	BUG_ON(ISP_BUFS_IS_FULL(bufs));

	buf = ISP_BUF_QUEUE(bufs);

	buf->isp_addr = bufs->isp_addr_capture[vb->i];
	buf->complete = complete;
	buf->vb = vb;
	buf->priv = priv;
	buf->vb_state = VIDEOBUF_DONE;
	buf->vb->state = VIDEOBUF_ACTIVE;

	if (ISP_BUFS_IS_EMPTY(bufs)) {
		isp_enable_interrupts(dev, RAW_CAPTURE(isp));
		isp_set_buf(dev, buf);
		ispccdc_enable(&isp->isp_ccdc, 1);
	}

	ISP_BUF_MARK_QUEUED(bufs);

	spin_unlock_irqrestore(&bufs->lock, flags);

	DPRINTK_ISPCTRL(KERN_ALERT "%s: queue %d vb %d, mmu %p\n", __func__,
			(bufs->queue - 1 + NUM_BUFS) % NUM_BUFS, vb->i,
			buf->isp_addr);

	return 0;
}
EXPORT_SYMBOL(isp_buf_queue);

/**
 * isp_vbq_setup - Do ISP specific actions when the VB wueue is set.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @vbq: Pointer to video buffer queue.
 * @cnt: Pointer to buffer count size of the queue list.
 * @size: Pointer to the bytesize of every video buffer queue entry.
 *
 * Currently, this just allocates the temporary buffer used for the
 * ISP Workaround when having CCDC->PRV->RSZ internal datapath.
 **/
int isp_vbq_setup(struct device *dev, struct videobuf_queue *vbq,
		  unsigned int *cnt, unsigned int *size)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	size_t tmp_size = PAGE_ALIGN(isp->pipeline.prv_out_w
				     * isp->pipeline.prv_out_h
				     * ISP_BYTES_PER_PIXEL);

	if (isp->pipeline.modules & OMAP_ISP_PREVIEW
	    && isp->tmp_buf_size < tmp_size)
		return isp_tmp_buf_alloc(dev, tmp_size);

	return 0;
}
EXPORT_SYMBOL(isp_vbq_setup);

/**
 * ispmmu_vmap - Wrapper for Virtual memory mapping of a scatter gather list
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @sglist: Pointer to source Scatter gather list to allocate.
 * @sglen: Number of elements of the scatter-gatter list.
 *
 * Returns a resulting mapped device address by the ISP MMU, or -ENOMEM if
 * we ran out of memory.
 **/
dma_addr_t ispmmu_vmap(struct device *dev, const struct scatterlist *sglist,
		       int sglen)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	int err;
	u32 da;
	struct sg_table *sgt;
	unsigned int i;
	struct scatterlist *sg, *src = (struct scatterlist *)sglist;

	/*
	 * convert isp sglist to iommu sgt
	 * FIXME: should be fixed in the upper layer?
	 */
	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return -ENOMEM;
	err = sg_alloc_table(sgt, sglen, GFP_KERNEL);
	if (err)
		goto err_sg_alloc;

	for_each_sg(sgt->sgl, sg, sgt->nents, i)
		sg_set_buf(sg, phys_to_virt(sg_dma_address(src + i)),
			   sg_dma_len(src + i));

	da = iommu_vmap(isp->iommu, 0, sgt, IOMMU_FLAG);
	if (IS_ERR_VALUE(da))
		goto err_vmap;

	return (dma_addr_t)da;

err_vmap:
	sg_free_table(sgt);
err_sg_alloc:
	kfree(sgt);
	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(ispmmu_vmap);

/**
 * ispmmu_vunmap - Unmap a device address from the ISP MMU
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @da: Device address generated from a ispmmu_vmap call.
 **/
void ispmmu_vunmap(struct device *dev, dma_addr_t da)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	struct sg_table *sgt;

	sgt = iommu_vunmap(isp->iommu, (u32)da);
	if (!sgt)
		return;
	sg_free_table(sgt);
	kfree(sgt);
}
EXPORT_SYMBOL_GPL(ispmmu_vunmap);

/**
 * isp_vbq_prepare - Videobuffer queue prepare.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @vbq: Pointer to videobuf_queue structure.
 * @vb: Pointer to videobuf_buffer structure.
 * @field: Requested Field order for the videobuffer.
 *
 * Returns 0 if successful, or -EIO if the ispmmu was unable to map a
 * scatter-gather linked list data space.
 **/
int isp_vbq_prepare(struct device *dev, struct videobuf_queue *vbq,
		    struct videobuf_buffer *vb, enum v4l2_field field)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	unsigned int isp_addr;
	struct videobuf_dmabuf *vdma;
	struct isp_bufs *bufs = &isp->bufs;

	int err = 0;

	vdma = videobuf_to_dma(vb);

	isp_addr = ispmmu_vmap(dev, vdma->sglist, vdma->sglen);

	if (IS_ERR_VALUE(isp_addr))
		err = -EIO;
	else
		bufs->isp_addr_capture[vb->i] = isp_addr;

	return err;
}
EXPORT_SYMBOL(isp_vbq_prepare);

/**
 * isp_vbq_release - Videobuffer queue release.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @vbq: Pointer to videobuf_queue structure.
 * @vb: Pointer to videobuf_buffer structure.
 **/
void isp_vbq_release(struct device *dev, struct videobuf_queue *vbq,
		     struct videobuf_buffer *vb)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	struct isp_bufs *bufs = &isp->bufs;

	ispmmu_vunmap(dev, bufs->isp_addr_capture[vb->i]);
	bufs->isp_addr_capture[vb->i] = (dma_addr_t)NULL;
	return;
}
EXPORT_SYMBOL(isp_vbq_release);

/**
 * isp_queryctrl - Query V4L2 control from existing controls in ISP.
 * @a: Pointer to v4l2_queryctrl structure. It only needs the id field filled.
 *
 * Returns 0 if successful, or -EINVAL if not found in ISP.
 **/
int isp_queryctrl(struct v4l2_queryctrl *a)
{
	int i;

	if (a->id & V4L2_CTRL_FLAG_NEXT_CTRL) {
		a->id &= ~V4L2_CTRL_FLAG_NEXT_CTRL;
		i = find_next_vctrl(a->id);
	} else {
		i = find_vctrl(a->id);
	}

	if (i < 0)
		return -EINVAL;

	*a = video_control[i].qc;
	return 0;
}
EXPORT_SYMBOL(isp_queryctrl);

/**
 * isp_queryctrl - Query V4L2 control from existing controls in ISP.
 * @a: Pointer to v4l2_queryctrl structure. It only needs the id field filled.
 *
 * Returns 0 if successful, or -EINVAL if not found in ISP.
 **/
int isp_querymenu(struct v4l2_querymenu *a)
{
	int i;

	i = find_vmenu(a->id, a->index);

	if (i < 0)
		return -EINVAL;

	*a = video_menu[i];
	return 0;
}
EXPORT_SYMBOL(isp_querymenu);

/**
 * isp_g_ctrl - Get value of the desired V4L2 control.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @a: V4L2 control to read actual value from.
 *
 * Return 0 if successful, or -EINVAL if chosen control is not found.
 **/
int isp_g_ctrl(struct device *dev, struct v4l2_control *a)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	u8 current_value;
	int rval = 0;

	if (!isp->ref_count)
		return -EINVAL;

	switch (a->id) {
	case V4L2_CID_BRIGHTNESS:
		isppreview_query_brightness(&isp->isp_prev, &current_value);
		a->value = current_value / ISPPRV_BRIGHT_UNITS;
		break;
	case V4L2_CID_CONTRAST:
		isppreview_query_contrast(&isp->isp_prev, &current_value);
		a->value = current_value / ISPPRV_CONTRAST_UNITS;
		break;
	case V4L2_CID_COLORFX:
		isppreview_get_color(&isp->isp_prev, &current_value);
		a->value = current_value;
		break;
	default:
		rval = -EINVAL;
		break;
	}

	return rval;
}
EXPORT_SYMBOL(isp_g_ctrl);

/**
 * isp_s_ctrl - Set value of the desired V4L2 control.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @a: V4L2 control to read actual value from.
 *
 * Return 0 if successful, -EINVAL if chosen control is not found or value
 * is out of bounds, -EFAULT if copy_from_user or copy_to_user operation fails
 * from camera abstraction layer related controls or the transfered user space
 * pointer via the value field is not set properly.
 **/
int isp_s_ctrl(struct device *dev, struct v4l2_control *a)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	int rval = 0;
	u8 new_value = a->value;

	if (!isp->ref_count)
		return -EINVAL;

	switch (a->id) {
	case V4L2_CID_BRIGHTNESS:
		if (a->value > ISPPRV_BRIGHT_HIGH)
			rval = -EINVAL;
		else
			isppreview_update_brightness(&isp->isp_prev,
						     &new_value);
		break;
	case V4L2_CID_CONTRAST:
		if (a->value > ISPPRV_CONTRAST_HIGH)
			rval = -EINVAL;
		else
			isppreview_update_contrast(&isp->isp_prev, &new_value);
		break;
	case V4L2_CID_COLORFX:
		if (a->value > V4L2_COLORFX_SEPIA)
			rval = -EINVAL;
		else
			isppreview_set_color(&isp->isp_prev, &new_value);
		break;
	default:
		rval = -EINVAL;
		break;
	}

	return rval;
}
EXPORT_SYMBOL(isp_s_ctrl);

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
int isp_handle_private(struct device *dev, int cmd, void *arg)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	int rval = 0;

	if (!isp->ref_count)
		return -EINVAL;

	switch (cmd) {
	case VIDIOC_PRIVATE_ISP_CCDC_CFG:
		rval = omap34xx_isp_ccdc_config(&isp->isp_ccdc, arg);
		break;
	case VIDIOC_PRIVATE_ISP_PRV_CFG:
		rval = omap34xx_isp_preview_config(&isp->isp_prev, arg);
		break;
	case VIDIOC_PRIVATE_ISP_AEWB_CFG: {
		struct isph3a_aewb_config *params;
		params = (struct isph3a_aewb_config *)arg;
		rval = isph3a_aewb_configure(&isp->isp_h3a, params);
	}
		break;
	case VIDIOC_PRIVATE_ISP_AEWB_REQ: {
		struct isph3a_aewb_data *data;
		data = (struct isph3a_aewb_data *)arg;
		rval = isph3a_aewb_request_statistics(&isp->isp_h3a, data);
	}
		break;
	case VIDIOC_PRIVATE_ISP_HIST_CFG: {
		struct isp_hist_config *params;
		params = (struct isp_hist_config *)arg;
		rval = isp_hist_configure(&isp->isp_hist, params);
	}
		break;
	case VIDIOC_PRIVATE_ISP_HIST_REQ: {
		struct isp_hist_data *data;
		data = (struct isp_hist_data *)arg;
		rval = isp_hist_request_statistics(&isp->isp_hist, data);
	}
		break;
	case VIDIOC_PRIVATE_ISP_AF_CFG: {
		struct af_configuration *params;
		params = (struct af_configuration *)arg;
		rval = isp_af_configure(&isp->isp_af, params);
	}
		break;
	case VIDIOC_PRIVATE_ISP_AF_REQ: {
		struct isp_af_data *data;
		data = (struct isp_af_data *)arg;
		rval = isp_af_request_statistics(&isp->isp_af, data);
	}
		break;
	default:
		rval = -EINVAL;
		break;
	}
	return rval;
}
EXPORT_SYMBOL(isp_handle_private);

/**
 * isp_enum_fmt_cap - Get more information of chosen format index and type
 * @f: Pointer to structure containing index and type of format to read from.
 *
 * Returns 0 if successful, or -EINVAL if format index or format type is
 * invalid.
 **/
int isp_enum_fmt_cap(struct v4l2_fmtdesc *f)
{
	int index = f->index;
	enum v4l2_buf_type type = f->type;
	int rval = -EINVAL;

	if (index >= NUM_ISP_CAPTURE_FORMATS)
		goto err;

	memset(f, 0, sizeof(*f));
	f->index = index;
	f->type = type;

	switch (f->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		rval = 0;
		break;
	default:
		goto err;
	}

	f->flags = isp_formats[index].flags;
	strncpy(f->description, isp_formats[index].description,
		sizeof(f->description));
	f->pixelformat = isp_formats[index].pixelformat;
err:
	return rval;
}
EXPORT_SYMBOL(isp_enum_fmt_cap);

/**
 * isp_g_fmt_cap - Get current output image format.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @pix: Pointer to V4L2 format structure to return current output format
 **/
void isp_g_fmt_cap(struct device *dev, struct v4l2_pix_format *pix)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	*pix = isp->pipeline.pix;
	return;
}
EXPORT_SYMBOL(isp_g_fmt_cap);

/**
 * isp_s_fmt_cap - Set I/O formats and crop, and configure pipeline in ISP
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @pix_input: Pointer to V4L2 format structure to represent current input.
 * @pix_output: Pointer to V4L2 format structure to represent current output.
 *
 * Returns 0 if successful, -EINVAL if ISP hasn't been opened, or return
 * value of isp_s_pipeline if there is an error.
 **/
int isp_s_fmt_cap(struct device *dev, struct v4l2_pix_format *pix_input,
		  struct v4l2_pix_format *pix_output)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	if (!isp->ref_count)
		return -EINVAL;

	return isp_s_pipeline(dev, pix_input, pix_output);
}
EXPORT_SYMBOL(isp_s_fmt_cap);

/**
 * isp_g_crop - Get crop rectangle size and position.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @crop: Pointer to V4L2 crop structure to be filled.
 *
 * Always returns 0.
 **/
int isp_g_crop(struct device *dev, struct v4l2_crop *crop)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	if (isp->pipeline.modules & OMAP_ISP_RESIZER) {
		crop->c = isp->pipeline.rsz_crop;
	} else {
		crop->c.left = 0;
		crop->c.top = 0;
		crop->c.width = isp->pipeline.ccdc_out_w_img;
		crop->c.height = isp->pipeline.ccdc_out_h;
	}

	return 0;
}
EXPORT_SYMBOL(isp_g_crop);

/**
 * isp_s_crop - Set crop rectangle size and position.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @a: Pointer to V4L2 crop structure with desired parameters.
 *
 * Always returns 0.
 *
 * FIXME: Hardcoded to configure always the resizer, which could not be always
 *        the case.
 **/
int isp_s_crop(struct device *dev, struct v4l2_crop *a)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	ispresizer_config_crop(&isp->isp_res, a);

	return 0;
}
EXPORT_SYMBOL(isp_s_crop);

/**
 * isp_try_fmt_cap - Try desired input/output image formats
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @pix_input: Pointer to V4L2 pixel format structure for input image.
 * @pix_output: Pointer to V4L2 pixel format structure for output image.
 *
 * Returns 0 if successful, or return value of either isp_try_size or
 * isp_try_fmt if there is an error.
 **/
int isp_try_fmt_cap(struct device *dev, struct v4l2_pix_format *pix_input,
		    struct v4l2_pix_format *pix_output)
{
	struct isp_pipeline pipe;
	int rval;

	pipe.pix = *pix_output;

	rval = isp_try_pipeline(dev, pix_input, &pipe);
	if (rval)
		return rval;

	*pix_output = pipe.pix;

	return 0;
}
EXPORT_SYMBOL(isp_try_fmt_cap);

/**
 * isp_save_ctx - Saves ISP, CCDC, HIST, H3A, PREV, RESZ & MMU context.
 * @dev: Device pointer specific to the OMAP3 ISP.
 *
 * Routine for saving the context of each module in the ISP.
 * CCDC, HIST, H3A, PREV, RESZ and IOMMU.
 **/
static void isp_save_ctx(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	isp_save_context(dev, isp_reg_list);
	ispccdc_save_context(dev);
	if (isp->iommu)
		iommu_save_ctx(isp->iommu);
	isphist_save_context(dev);
	isph3a_save_context(dev);
	isppreview_save_context(dev);
	ispresizer_save_context(dev);
	ispcsi2_save_context(dev);
}

/**
 * isp_restore_ctx - Restores ISP, CCDC, HIST, H3A, PREV, RESZ & MMU context.
 * @dev: Device pointer specific to the OMAP3 ISP.
 *
 * Routine for restoring the context of each module in the ISP.
 * CCDC, HIST, H3A, PREV, RESZ and IOMMU.
 **/
static void isp_restore_ctx(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	isp_restore_context(dev, isp_reg_list);
	ispccdc_restore_context(dev);
	if (isp->iommu)
		iommu_restore_ctx(isp->iommu);
	isphist_restore_context(dev);
	isph3a_restore_context(dev);
	isppreview_restore_context(dev);
	ispresizer_restore_context(dev);
	ispcsi2_restore_context(dev);
}

/**
 * isp_enable_clocks - Enable ISP clocks
 * @dev: Device pointer specific to the OMAP3 ISP.
 *
 * Return 0 if successful, or clk_enable return value if any of tthem fails.
 **/
static int isp_enable_clocks(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);
	int r;

	r = clk_enable(isp->cam_ick);
	if (r) {
		dev_err(dev, "clk_enable cam_ick failed\n");
		goto out_clk_enable_ick;
	}
	r = clk_enable(isp->cam_mclk);
	if (r) {
		dev_err(dev, "clk_enable cam_mclk failed\n");
		goto out_clk_enable_mclk;
	}
	r = clk_enable(isp->csi2_fck);
	if (r) {
		dev_err(dev, "clk_enable csi2_fck failed\n");
		goto out_clk_enable_csi2_fclk;
	}
	return 0;

out_clk_enable_csi2_fclk:
	clk_disable(isp->cam_mclk);
out_clk_enable_mclk:
	clk_disable(isp->cam_ick);
out_clk_enable_ick:
	return r;
}

/**
 * isp_disable_clocks - Disable ISP clocks
 * @dev: Device pointer specific to the OMAP3 ISP.
 **/
static void isp_disable_clocks(struct device *dev)
{
	struct isp_device *isp = dev_get_drvdata(dev);

	clk_disable(isp->cam_ick);
	clk_disable(isp->cam_mclk);
	clk_disable(isp->csi2_fck);
}

/**
 * isp_get - Acquire the ISP resource.
 *
 * Initializes the clocks for the first acquire.
 *
 * Returns pointer for isp device structure.
 **/
struct device *isp_get(void)
{
	struct platform_device *pdev = omap3isp_pdev;
	struct isp_device *isp;
	static int has_context;
	int ret_err = 0;

	if (!pdev)
		return NULL;
	isp = platform_get_drvdata(pdev);

	DPRINTK_ISPCTRL("isp_get: old %d\n", isp->ref_count);
	mutex_lock(&(isp->isp_mutex));
	if (isp->ref_count == 0) {
		ret_err = isp_enable_clocks(&pdev->dev);
		if (ret_err)
			goto out_err;
		/* We don't want to restore context before saving it! */
		if (has_context)
			isp_restore_ctx(&pdev->dev);
		else
			has_context = 1;
	}
	isp->ref_count++;
	mutex_unlock(&(isp->isp_mutex));

	DPRINTK_ISPCTRL("isp_get: new %d\n", isp->ref_count);
	/* FIXME: ISP should register as v4l2 device to store its priv data */
	return &pdev->dev;

out_err:
	mutex_unlock(&(isp->isp_mutex));
	return NULL;
}
EXPORT_SYMBOL(isp_get);

/**
 * isp_put - Release the ISP resource.
 *
 * Releases the clocks also for the last release.
 *
 * Return resulting reference count, or -EBUSY if ISP structure is not
 * allocated.
 **/
int isp_put(void)
{
	struct platform_device *pdev = omap3isp_pdev;
	struct isp_device *isp = platform_get_drvdata(pdev);

	if (!isp)
		return -EBUSY;

	DPRINTK_ISPCTRL("isp_put: old %d\n", isp->ref_count);
	mutex_lock(&(isp->isp_mutex));
	if (isp->ref_count) {
		if (--isp->ref_count == 0) {
			isp_save_ctx(&pdev->dev);
			isp_tmp_buf_free(&pdev->dev);
			isp_release_resources(&pdev->dev);
			isp_disable_clocks(&pdev->dev);
		}
	}
	mutex_unlock(&(isp->isp_mutex));
	DPRINTK_ISPCTRL("isp_put: new %d\n", isp->ref_count);
	return isp->ref_count;
}
EXPORT_SYMBOL(isp_put);

/**
 * isp_save_context - Saves the values of the ISP module registers.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @reg_list: Structure containing pairs of register address and value to
 *            modify on OMAP.
 **/
void isp_save_context(struct device *dev, struct isp_reg *reg_list)
{
	struct isp_reg *next = reg_list;

	for (; next->reg != ISP_TOK_TERM; next++)
		next->val = isp_reg_readl(dev, next->mmio_range, next->reg);
}
EXPORT_SYMBOL(isp_save_context);

/**
 * isp_restore_context - Restores the values of the ISP module registers.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @reg_list: Structure containing pairs of register address and value to
 *            modify on OMAP.
 **/
void isp_restore_context(struct device *dev, struct isp_reg *reg_list)
{
	struct isp_reg *next = reg_list;

	for (; next->reg != ISP_TOK_TERM; next++)
		isp_reg_writel(dev, next->val, next->mmio_range, next->reg);
}
EXPORT_SYMBOL(isp_restore_context);

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

	if (!isp)
		return 0;

	isp_csi2_cleanup(&pdev->dev);
	isp_af_exit(&pdev->dev);
	isp_preview_cleanup(&pdev->dev);
	isp_resizer_cleanup(&pdev->dev);
	isp_get();
	if (isp->iommu)
		iommu_put(isp->iommu);
	isp_put();
	isph3a_aewb_cleanup(&pdev->dev);
	isp_hist_cleanup(&pdev->dev);
	isp_ccdc_cleanup(&pdev->dev);

	clk_put(isp->cam_ick);
	clk_put(isp->cam_mclk);
	clk_put(isp->csi2_fck);
	clk_put(isp->l3_ick);

	free_irq(isp->irq_num, isp);

	for (i = 0; i <= OMAP3_ISP_IOMEM_CSI2PHY; i++) {
		if (isp->mmio_base[i]) {
			iounmap((void *)isp->mmio_base[i]);
			isp->mmio_base[i] = 0;
		}

		if (isp->mmio_base_phys[i]) {
			release_mem_region(isp->mmio_base_phys[i],
					   isp->mmio_size[i]);
			isp->mmio_base_phys[i] = 0;
		}
	}

	omap3isp_pdev = NULL;
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

	DPRINTK_ISPCTRL("isp_suspend: starting\n");

	if (mutex_is_locked(&isp->isp_mutex))
		dev_err(dev, "%s: bug: isp_mutex is locked\n", __func__);

	if (isp->ref_count == 0)
		goto out;

	isp_disable_interrupts(dev);
	reset = isp_suspend_modules(dev);
	isp_save_ctx(dev);
	if (reset)
		isp_reset(dev);

	isp_disable_clocks(dev);

out:
	DPRINTK_ISPCTRL("isp_suspend: done\n");

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

	DPRINTK_ISPCTRL("isp_resume: starting\n");

	if (mutex_is_locked(&isp->isp_mutex))
		dev_err(dev, "%s: bug: isp_mutex is locked\n", __func__);

	if (isp->ref_count == 0)
		goto out;

	ret_err = isp_enable_clocks(dev);
	if (ret_err)
		goto out;
	isp_restore_ctx(dev);
	isp_resume_modules(dev);

out:
	DPRINTK_ISPCTRL("isp_resume: done \n");

	return ret_err;
}

#else

#define isp_suspend	NULL
#define isp_resume	NULL

#endif /* CONFIG_PM */

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
	struct isp_device *isp;
	int ret_err = 0;
	int i;

	isp = kzalloc(sizeof(*isp), GFP_KERNEL);
	if (!isp) {
		dev_err(&pdev->dev, "could not allocate memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, isp);

	isp->dev = &pdev->dev;

	for (i = 0; i <= OMAP3_ISP_IOMEM_CSI2PHY; i++) {
		struct resource *mem;
		/* request the mem region for the camera registers */
		mem = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!mem) {
			dev_err(isp->dev, "no mem resource?\n");
			ret_err = -ENODEV;
			goto out_free_mmio;
		}

		if (!request_mem_region(mem->start, mem->end - mem->start + 1,
					pdev->name)) {
			dev_err(isp->dev,
				"cannot reserve camera register I/O region\n");
			ret_err = -ENODEV;
			goto out_free_mmio;
		}
		isp->mmio_base_phys[i] = mem->start;
		isp->mmio_size[i] = mem->end - mem->start + 1;

		/* map the region */
		isp->mmio_base[i] = (unsigned long)
			ioremap_nocache(isp->mmio_base_phys[i],
					isp->mmio_size[i]);
		if (!isp->mmio_base[i]) {
			dev_err(isp->dev,
				"cannot map camera register I/O region\n");
			ret_err = -ENODEV;
			goto out_free_mmio;
		}
	}

	isp->irq_num = platform_get_irq(pdev, 0);
	if (isp->irq_num <= 0) {
		dev_err(isp->dev, "no irq for camera?\n");
		ret_err = -ENODEV;
		goto out_free_mmio;
	}

	isp->cam_ick = clk_get(&camera_dev, "cam_ick");
	if (IS_ERR(isp->cam_ick)) {
		dev_err(isp->dev, "clk_get cam_ick failed\n");
		ret_err = PTR_ERR(isp->cam_ick);
		goto out_free_mmio;
	}
	isp->cam_mclk = clk_get(&camera_dev, "cam_mclk");
	if (IS_ERR(isp->cam_mclk)) {
		dev_err(isp->dev, "clk_get cam_mclk failed\n");
		ret_err = PTR_ERR(isp->cam_mclk);
		goto out_clk_get_mclk;
	}
	isp->csi2_fck = clk_get(&camera_dev, "csi2_96m_fck");
	if (IS_ERR(isp->csi2_fck)) {
		dev_err(isp->dev, "clk_get csi2_96m_fck failed\n");
		ret_err = PTR_ERR(isp->csi2_fck);
		goto out_clk_get_csi2_fclk;
	}
	isp->l3_ick = clk_get(&camera_dev, "l3_ick");
	if (IS_ERR(isp->l3_ick)) {
		dev_err(isp->dev, "clk_get l3_ick failed\n");
		ret_err = PTR_ERR(isp->l3_ick);
		goto out_clk_get_l3_ick;
	}

	if (request_irq(isp->irq_num, omap34xx_isp_isr, IRQF_SHARED,
			"Omap 3 Camera ISP", pdev)) {
		dev_err(isp->dev, "could not install isr\n");
		ret_err = -EINVAL;
		goto out_request_irq;
	}

	isp->ref_count = 0;
	omap3isp_pdev = pdev;

	mutex_init(&(isp->isp_mutex));
	spin_lock_init(&isp->lock);
	spin_lock_init(&isp->bufs.lock);
	spin_lock_init(&isp->h3a_lock);

	isp_get();
	isp->iommu = iommu_get("isp");
	if (IS_ERR(isp->iommu)) {
		ret_err = PTR_ERR(isp->iommu);
		isp->iommu = NULL;
	}
	isp_put();
	if (!isp->iommu)
		goto out_iommu_get;

	isp_ccdc_init(&pdev->dev);
	isp_hist_init(&pdev->dev);
	isph3a_aewb_init(&pdev->dev);
	isp_preview_init(&pdev->dev);
	isp_resizer_init(&pdev->dev);
	isp_af_init(&pdev->dev);
	isp_csi2_init(&pdev->dev);

	isp_get();
	isp_power_settings(&pdev->dev, 1);
	isp_put();

	return 0;

out_iommu_get:
	free_irq(isp->irq_num, isp);
	omap3isp_pdev = NULL;
out_request_irq:
	clk_put(isp->l3_ick);
out_clk_get_l3_ick:
	clk_put(isp->csi2_fck);
out_clk_get_csi2_fclk:
	clk_put(isp->cam_mclk);
out_clk_get_mclk:
	clk_put(isp->cam_ick);
out_free_mmio:
	for (i = 0; i <= OMAP3_ISP_IOMEM_CSI2PHY; i++) {
		if (isp->mmio_base[i]) {
			iounmap((void *)isp->mmio_base[i]);
			isp->mmio_base[i] = 0;
		}

		if (isp->mmio_base_phys[i]) {
			release_mem_region(isp->mmio_base_phys[i],
					   isp->mmio_size[i]);
			isp->mmio_base_phys[i] = 0;
		}
	}

	kfree(isp);
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
void isp_print_status(struct device *dev)
{
	if (!is_ispctrl_debug_enabled())
		return;

	DPRINTK_ISPCTRL("###ISP_CTRL=0x%x\n",
			isp_reg_readl(dev, OMAP3_ISP_IOMEM_MAIN, ISP_CTRL));
	DPRINTK_ISPCTRL("###ISP_TCTRL_CTRL=0x%x\n",
			isp_reg_readl(dev, OMAP3_ISP_IOMEM_MAIN,
				      ISP_TCTRL_CTRL));
	DPRINTK_ISPCTRL("###ISP_SYSCONFIG=0x%x\n",
			isp_reg_readl(dev, OMAP3_ISP_IOMEM_MAIN,
				      ISP_SYSCONFIG));
	DPRINTK_ISPCTRL("###ISP_SYSSTATUS=0x%x\n",
			isp_reg_readl(dev, OMAP3_ISP_IOMEM_MAIN,
				      ISP_SYSSTATUS));
	DPRINTK_ISPCTRL("###ISP_IRQ0ENABLE=0x%x\n",
			isp_reg_readl(dev, OMAP3_ISP_IOMEM_MAIN,
				      ISP_IRQ0ENABLE));
	DPRINTK_ISPCTRL("###ISP_IRQ0STATUS=0x%x\n",
			isp_reg_readl(dev, OMAP3_ISP_IOMEM_MAIN,
				      ISP_IRQ0STATUS));
}
EXPORT_SYMBOL(isp_print_status);

module_init(isp_init);
module_exit(isp_cleanup);

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("ISP Control Module Library");
MODULE_LICENSE("GPL");
