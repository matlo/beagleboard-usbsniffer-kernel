/*
 * isph3a_af.c
 *
 * AF module for TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contributors:
 *	Sergio Aguirre <saaguirre@ti.com>
 *	Troy Laramy
 * 	David Cohen <david.cohen@nokia.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Linux specific include files */
#include <linux/device.h>

#include "isp.h"
#include "isph3a.h"
#include "ispstat.h"

#define IS_OUT_OF_BOUNDS(value, min, max)		\
	(((value) < (min)) || ((value) > (max)))

static void isph3a_af_setup_regs(struct ispstat *af)
{
	struct isph3a_af_config *conf = af->priv;
	u32 pcr;
	u32 pax1;
	u32 pax2;
	u32 paxstart;
	u32 coef;
	u32 base_coef_set0;
	u32 base_coef_set1;
	int index;

	if (af->state == ISPSTAT_DISABLED)
		return;

	isp_reg_writel(af->isp, af->active_buf->iommu_addr, OMAP3_ISP_IOMEM_H3A,
		       ISPH3A_AFBUFST);

	if (!af->update)
		return;

	/* Configure Hardware Registers */
	pax1 = ((conf->paxel.width >> 1) - 1) << AF_PAXW_SHIFT;
	/* Set height in AFPAX1 */
	pax1 |= (conf->paxel.height >> 1) - 1;
	isp_reg_writel(af->isp, pax1, OMAP3_ISP_IOMEM_H3A, ISPH3A_AFPAX1);

	/* Configure AFPAX2 Register */
	/* Set Line Increment in AFPAX2 Register */
	pax2 = ((conf->paxel.line_inc >> 1) - 1) << AF_LINE_INCR_SHIFT;
	/* Set Vertical Count */
	pax2 |= (conf->paxel.v_cnt - 1) << AF_VT_COUNT_SHIFT;
	/* Set Horizontal Count */
	pax2 |= (conf->paxel.h_cnt - 1);
	isp_reg_writel(af->isp, pax2, OMAP3_ISP_IOMEM_H3A, ISPH3A_AFPAX2);

	/* Configure PAXSTART Register */
	/*Configure Horizontal Start */
	paxstart = conf->paxel.h_start << AF_HZ_START_SHIFT;
	/* Configure Vertical Start */
	paxstart |= conf->paxel.v_start;
	isp_reg_writel(af->isp, paxstart, OMAP3_ISP_IOMEM_H3A,
		       ISPH3A_AFPAXSTART);

	/*SetIIRSH Register */
	isp_reg_writel(af->isp, conf->iir.h_start,
		       OMAP3_ISP_IOMEM_H3A, ISPH3A_AFIIRSH);

	base_coef_set0 = ISPH3A_AFCOEF010;
	base_coef_set1 = ISPH3A_AFCOEF110;
	for (index = 0; index <= 8; index += 2) {
		/*Set IIR Filter0 Coefficients */
		coef = 0;
		coef |= conf->iir.coeff_set0[index];
		coef |= conf->iir.coeff_set0[index + 1] <<
			AF_COEF_SHIFT;
		isp_reg_writel(af->isp, coef, OMAP3_ISP_IOMEM_H3A,
			       base_coef_set0);
		base_coef_set0 += AFCOEF_OFFSET;

		/*Set IIR Filter1 Coefficients */
		coef = 0;
		coef |= conf->iir.coeff_set1[index];
		coef |= conf->iir.coeff_set1[index + 1] <<
			AF_COEF_SHIFT;
		isp_reg_writel(af->isp, coef, OMAP3_ISP_IOMEM_H3A,
			       base_coef_set1);
		base_coef_set1 += AFCOEF_OFFSET;
	}
	/* set AFCOEF0010 Register */
	isp_reg_writel(af->isp, conf->iir.coeff_set0[10],
		       OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF0010);
	/* set AFCOEF1010 Register */
	isp_reg_writel(af->isp, conf->iir.coeff_set1[10],
		       OMAP3_ISP_IOMEM_H3A, ISPH3A_AFCOEF1010);

	/* PCR Register */
	/* Set RGB Position */
	pcr = conf->rgb_pos << AF_RGBPOS_SHIFT;
	/* Set Accumulator Mode */
	if (conf->fvmode == AF_MODE_PEAK)
		pcr |= AF_FVMODE;
	/* Set A-law */
	if (conf->alaw_enable)
		pcr |= AF_ALAW_EN;
	/* HMF Configurations */
	if (conf->hmf.enable) {
		/* Enable HMF */
		pcr |= AF_MED_EN;
		/* Set Median Threshold */
		pcr |= conf->hmf.threshold << AF_MED_TH_SHIFT;
	}
	/* Set PCR Register */
	isp_reg_and_or(af->isp, OMAP3_ISP_IOMEM_H3A, ISPH3A_PCR,
		       ~AF_PCR_MASK, pcr);

	af->update = 0;
	af->config_counter += af->inc_config;
	af->inc_config = 0;
	af->buf_size = conf->buf_size;
}

static u32 isph3a_af_get_buf_size(struct isph3a_af_config *conf)
{
	return conf->paxel.h_cnt * conf->paxel.v_cnt * AF_PAXEL_SIZE;
}

/* Function to check paxel parameters */
static int isph3a_af_validate_params(struct ispstat *af, void *new_conf)
{
	struct isph3a_af_config *user_cfg = new_conf;
	struct isph3a_af_paxel *paxel_cfg = &user_cfg->paxel;
	struct isph3a_af_iir *iir_cfg = &user_cfg->iir;
	int index;
	u32 buf_size;

	/* Check horizontal Count */
	if (IS_OUT_OF_BOUNDS(paxel_cfg->h_cnt, AF_PAXEL_HORIZONTAL_COUNT_MIN,
			     AF_PAXEL_HORIZONTAL_COUNT_MAX))
		return -EINVAL;

	/* Check Vertical Count */
	if (IS_OUT_OF_BOUNDS(paxel_cfg->v_cnt, AF_PAXEL_VERTICAL_COUNT_MIN,
			     AF_PAXEL_VERTICAL_COUNT_MAX))
		return -EINVAL;

	if (IS_OUT_OF_BOUNDS(paxel_cfg->height, AF_PAXEL_HEIGHT_MIN,
			     AF_PAXEL_HEIGHT_MAX) || paxel_cfg->height % 2)
		return -EINVAL;

	/* Check width */
	if (IS_OUT_OF_BOUNDS(paxel_cfg->width, AF_PAXEL_WIDTH_MIN,
			     AF_PAXEL_WIDTH_MAX) || paxel_cfg->width % 2)
		return -EINVAL;

	/* Check Line Increment */
	if (IS_OUT_OF_BOUNDS(paxel_cfg->line_inc, AF_PAXEL_INCREMENT_MIN,
			     AF_PAXEL_INCREMENT_MAX) || paxel_cfg->line_inc % 2)
		return -EINVAL;

	/* Check Horizontal Start */
	if ((paxel_cfg->h_start % 2 != 0) ||
	    (paxel_cfg->h_start < (iir_cfg->h_start + 2)) ||
	    IS_OUT_OF_BOUNDS(paxel_cfg->h_start,
			     AF_PAXEL_HZSTART_MIN, AF_PAXEL_HZSTART_MAX))
		return -EINVAL;

	/* Check IIR */
	for (index = 0; index < AF_NUM_COEF; index++) {
		if ((iir_cfg->coeff_set0[index]) > AF_COEF_MAX)
			return -EINVAL;

		if ((iir_cfg->coeff_set1[index]) > AF_COEF_MAX)
			return -EINVAL;
	}

	if (IS_OUT_OF_BOUNDS(iir_cfg->h_start, AF_IIRSH_MIN, AF_IIRSH_MAX))
		return -EINVAL;

	/* Hack: If paxel size is 12, the 10th AF window may be corrupted */
	if ((paxel_cfg->h_cnt * paxel_cfg->v_cnt > 9) &&
	    (paxel_cfg->width * paxel_cfg->height == 12))
		return -EINVAL;

	buf_size = isph3a_af_get_buf_size(user_cfg);
	if (buf_size > user_cfg->buf_size)
		/* User buf_size request wasn't enough */
		user_cfg->buf_size = buf_size;
	else if (user_cfg->buf_size > AF_MAX_BUF_SIZE)
		user_cfg->buf_size = AF_MAX_BUF_SIZE;

	return 0;
}

/* Update local parameters */
static void isph3a_af_set_params(struct ispstat *af, void *new_conf)
{
	struct isph3a_af_config *user_cfg = new_conf;
	struct isph3a_af_config *cur_cfg = af->priv;
	int update = 0;
	int index;

	/* alaw */
	if (cur_cfg->alaw_enable != user_cfg->alaw_enable) {
		update = 1;
		goto out;
	}

	/* hmf */
	if (cur_cfg->hmf.enable != user_cfg->hmf.enable) {
		update = 1;
		goto out;
	}
	if (cur_cfg->hmf.threshold != user_cfg->hmf.threshold) {
		update = 1;
		goto out;
	}

	/* rgbpos */
	if (cur_cfg->rgb_pos != user_cfg->rgb_pos) {
		update = 1;
		goto out;
	}

	/* iir */
	if (cur_cfg->iir.h_start != user_cfg->iir.h_start) {
		update = 1;
		goto out;
	}
	for (index = 0; index < AF_NUM_COEF; index++) {
		if (cur_cfg->iir.coeff_set0[index] !=
				user_cfg->iir.coeff_set0[index]) {
			update = 1;
			goto out;
		}
		if (cur_cfg->iir.coeff_set1[index] !=
				user_cfg->iir.coeff_set1[index]) {
			update = 1;
			goto out;
		}
	}

	/* paxel */
	if ((cur_cfg->paxel.width != user_cfg->paxel.width) ||
	    (cur_cfg->paxel.height != user_cfg->paxel.height) ||
	    (cur_cfg->paxel.h_start != user_cfg->paxel.h_start) ||
	    (cur_cfg->paxel.v_start != user_cfg->paxel.v_start) ||
	    (cur_cfg->paxel.h_cnt != user_cfg->paxel.h_cnt) ||
	    (cur_cfg->paxel.v_cnt != user_cfg->paxel.v_cnt) ||
	    (cur_cfg->paxel.line_inc != user_cfg->paxel.line_inc)) {
		update = 1;
		goto out;
	}

	/* af_mode */
	if (cur_cfg->fvmode != user_cfg->fvmode)
		update = 1;

out:
	if (update || !af->configured) {
		memcpy(cur_cfg, user_cfg, sizeof(*cur_cfg));
		af->inc_config++;
		af->update = 1;
		/*
		 * User might be asked for a bigger buffer than necessary for
		 * this configuration. In order to return the right amount of
		 * data during buffer request, let's calculate the size here
		 * instead of stick with user_cfg->buf_size.
		 */
		cur_cfg->buf_size = isph3a_af_get_buf_size(cur_cfg);
	}
}

static long isph3a_af_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ispstat *stat = v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_PRIVATE_ISP_AF_CFG:
		return ispstat_config(stat, arg);
	case VIDIOC_PRIVATE_ISP_STAT_REQ:
		return ispstat_request_statistics(stat, arg);
	case VIDIOC_PRIVATE_ISP_STAT_EN: {
		int *en = arg;
		return ispstat_enable(stat, !!*en);
	}
	}

	return -ENOIOCTLCMD;

}

static const struct ispstat_ops isph3a_af_ops = {
	.validate_params = isph3a_af_validate_params,
	.set_params = isph3a_af_set_params,
	.setup_regs = isph3a_af_setup_regs,
};

static const struct ispstat_pcr_bits isph3a_af_pcr = {
	.base = OMAP3_ISP_IOMEM_H3A,
	.offset = ISPH3A_PCR,
	.enable = ISPH3A_PCR_AF_EN,
	.busy = ISPH3A_PCR_BUSYAF,
};

static const struct v4l2_subdev_core_ops isph3a_af_subdev_core_ops = {
	.ioctl = isph3a_af_ioctl,
	.subscribe_event = ispstat_subscribe_event,
	.unsubscribe_event = ispstat_unsubscribe_event,
};

static const struct v4l2_subdev_video_ops isph3a_af_subdev_video_ops = {
	.s_stream = ispstat_s_stream,
};

static const struct v4l2_subdev_ops isph3a_af_subdev_ops = {
	.core = &isph3a_af_subdev_core_ops,
	.video = &isph3a_af_subdev_video_ops,
};

/* Function to register the AF character device driver. */
int isph3a_af_init(struct isp_device *isp)
{
	struct ispstat *af = &isp->isp_af;
	struct isph3a_af_config *af_cfg;

	af_cfg = kzalloc(sizeof(*af_cfg), GFP_KERNEL);
	if (af_cfg == NULL)
		return -ENOMEM;

	memset(af, 0, sizeof(*af));
	af->ops = &isph3a_af_ops;
	af->pcr = &isph3a_af_pcr;
	af->priv = af_cfg;
	af->dma_ch = -1;
	af->event_type = V4L2_EVENT_OMAP3ISP_AF;
	af->isp = isp;

	return ispstat_init(af, "AF", &isph3a_af_subdev_ops);
}

void isph3a_af_cleanup(struct isp_device *isp)
{
	kfree(isp->isp_af.priv);
	ispstat_free(&isp->isp_af);
}
