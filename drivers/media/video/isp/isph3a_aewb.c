/*
 * isph3a.c
 *
 * H3A module for TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contributors:
 * 	David Cohen <david.cohen@nokia.com>
 * 	Sakari Ailus <sakari.ailus@nokia.com>
 *	Sergio Aguirre <saaguirre@ti.com>
 *	Troy Laramy
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <linux/uaccess.h>
#include <linux/slab.h>

#include "isp.h"
#include "isph3a.h"
#include "ispstat.h"

/**
 * isph3a_aewb_update_regs - Helper function to update h3a registers.
 **/
static void isph3a_aewb_setup_regs(struct ispstat *aewb)
{
	struct isph3a_aewb_config *conf = aewb->priv;
	u32 pcr;
	u32 win1;
	u32 start;
	u32 blk;
	u32 subwin;

	if (aewb->state == ISPSTAT_DISABLED)
		return;

	isp_reg_writel(aewb->isp, aewb->active_buf->iommu_addr,
		       OMAP3_ISP_IOMEM_H3A, ISPH3A_AEWBUFST);

	if (!aewb->update)
		return;

	/* Converting config metadata into reg values */
	pcr = conf->saturation_limit << ISPH3A_PCR_AEW_AVE2LMT_SHIFT;
	pcr |= !!conf->alaw_enable << ISPH3A_PCR_AEW_ALAW_EN_SHIFT;

	win1 = ((conf->win_height >> 1) - 1) << ISPH3A_AEWWIN1_WINH_SHIFT;
	win1 |= ((conf->win_width >> 1) - 1) << ISPH3A_AEWWIN1_WINW_SHIFT;
	win1 |= (conf->ver_win_count - 1) << ISPH3A_AEWWIN1_WINVC_SHIFT;
	win1 |= (conf->hor_win_count - 1) << ISPH3A_AEWWIN1_WINHC_SHIFT;

	start = conf->hor_win_start << ISPH3A_AEWINSTART_WINSH_SHIFT;
	start |= conf->ver_win_start << ISPH3A_AEWINSTART_WINSV_SHIFT;

	blk = conf->blk_ver_win_start << ISPH3A_AEWINBLK_WINSV_SHIFT;
	blk |= ((conf->blk_win_height >> 1) - 1) << ISPH3A_AEWINBLK_WINH_SHIFT;

	subwin = ((conf->subsample_ver_inc >> 1) - 1) <<
		 ISPH3A_AEWSUBWIN_AEWINCV_SHIFT;
	subwin |= ((conf->subsample_hor_inc >> 1) - 1) <<
		  ISPH3A_AEWSUBWIN_AEWINCH_SHIFT;

	isp_reg_writel(aewb->isp, win1, OMAP3_ISP_IOMEM_H3A, ISPH3A_AEWWIN1);
	isp_reg_writel(aewb->isp, start, OMAP3_ISP_IOMEM_H3A,
		       ISPH3A_AEWINSTART);
	isp_reg_writel(aewb->isp, blk, OMAP3_ISP_IOMEM_H3A, ISPH3A_AEWINBLK);
	isp_reg_writel(aewb->isp, subwin, OMAP3_ISP_IOMEM_H3A,
		       ISPH3A_AEWSUBWIN);
	isp_reg_and_or(aewb->isp, OMAP3_ISP_IOMEM_H3A, ISPH3A_PCR,
		       ~ISPH3A_PCR_AEW_MASK, pcr);

	aewb->update = 0;
	aewb->config_counter += aewb->inc_config;
	aewb->inc_config = 0;
	aewb->buf_size = conf->buf_size;
}

static u32 isph3a_aewb_get_buf_size(struct isph3a_aewb_config *conf)
{
	/* Number of configured windows + extra row for black data */
	u32 win_count = (conf->ver_win_count + 1) * conf->hor_win_count;

	/*
	 * Unsaturated block counts for each 8 windows.
	 * 1 extra for the last (win_count % 8) windows if win_count is not
	 * divisible by 8.
	 */
	win_count += (win_count + 7) / 8;

	return win_count * AEWB_PACKET_SIZE;
}

static int isph3a_aewb_validate_params(struct ispstat *aewb, void *new_conf)
{
	struct isph3a_aewb_config *user_cfg = new_conf;
	u32 buf_size;

	if (unlikely(user_cfg->saturation_limit > AEWB_MAX_SATURATION_LIM))
		return -EINVAL;

	if (unlikely(user_cfg->win_height < AEWB_MIN_WIN_H ||
		     user_cfg->win_height > AEWB_MAX_WIN_H ||
		     user_cfg->win_height & 0x01))
		return -EINVAL;

	if (unlikely(user_cfg->win_width < AEWB_MIN_WIN_W ||
		     user_cfg->win_width > AEWB_MAX_WIN_W ||
		     user_cfg->win_width & 0x01))
		return -EINVAL;

	if (unlikely(user_cfg->ver_win_count < 1 ||
		     user_cfg->ver_win_count > AEWB_MAX_WINVC))
		return -EINVAL;

	if (unlikely(user_cfg->hor_win_count < 1 ||
		     user_cfg->hor_win_count > AEWB_MAX_WINHC))
		return -EINVAL;

	if (unlikely(user_cfg->ver_win_start > AEWB_MAX_WINSTART))
		return -EINVAL;

	if (unlikely(user_cfg->hor_win_start > AEWB_MAX_WINSTART))
		return -EINVAL;

	if (unlikely(user_cfg->blk_ver_win_start > AEWB_MAX_WINSTART))
		return -EINVAL;

	if (unlikely(user_cfg->blk_win_height < AEWB_MIN_WIN_H ||
		     user_cfg->blk_win_height > AEWB_MAX_WIN_H ||
		     user_cfg->blk_win_height & 0x01))
		return -EINVAL;

	if (unlikely(user_cfg->subsample_ver_inc < AEWB_MIN_SUB_INC ||
		     user_cfg->subsample_ver_inc > AEWB_MAX_SUB_INC ||
		     user_cfg->subsample_ver_inc & 0x01))
		return -EINVAL;

	if (unlikely(user_cfg->subsample_hor_inc < AEWB_MIN_SUB_INC ||
		     user_cfg->subsample_hor_inc > AEWB_MAX_SUB_INC ||
		     user_cfg->subsample_hor_inc & 0x01))
		return -EINVAL;

	buf_size = isph3a_aewb_get_buf_size(user_cfg);
	if (buf_size > user_cfg->buf_size)
		user_cfg->buf_size = buf_size;
	else if (user_cfg->buf_size > AEWB_MAX_BUF_SIZE)
		user_cfg->buf_size = AEWB_MAX_BUF_SIZE;

	return 0;
}

/**
 * isph3a_aewb_set_params - Helper function to check & store user given params.
 * @new_conf: Pointer to AE and AWB parameters struct.
 *
 * As most of them are busy-lock registers, need to wait until AEW_BUSY = 0 to
 * program them during ISR.
 **/
static void isph3a_aewb_set_params(struct ispstat *aewb, void *new_conf)
{
	struct isph3a_aewb_config *user_cfg = new_conf;
	struct isph3a_aewb_config *cur_cfg = aewb->priv;
	int update = 0;

	if (cur_cfg->saturation_limit != user_cfg->saturation_limit) {
		cur_cfg->saturation_limit = user_cfg->saturation_limit;
		update = 1;
	}
	if (cur_cfg->alaw_enable != user_cfg->alaw_enable) {
		cur_cfg->alaw_enable = user_cfg->alaw_enable;
		update = 1;
	}
	if (cur_cfg->win_height != user_cfg->win_height) {
		cur_cfg->win_height = user_cfg->win_height;
		update = 1;
	}
	if (cur_cfg->win_width != user_cfg->win_width) {
		cur_cfg->win_width = user_cfg->win_width;
		update = 1;
	}
	if (cur_cfg->ver_win_count != user_cfg->ver_win_count) {
		cur_cfg->ver_win_count = user_cfg->ver_win_count;
		update = 1;
	}
	if (cur_cfg->hor_win_count != user_cfg->hor_win_count) {
		cur_cfg->hor_win_count = user_cfg->hor_win_count;
		update = 1;
	}
	if (cur_cfg->ver_win_start != user_cfg->ver_win_start) {
		cur_cfg->ver_win_start = user_cfg->ver_win_start;
		update = 1;
	}
	if (cur_cfg->hor_win_start != user_cfg->hor_win_start) {
		cur_cfg->hor_win_start = user_cfg->hor_win_start;
		update = 1;
	}
	if (cur_cfg->blk_ver_win_start != user_cfg->blk_ver_win_start) {
		cur_cfg->blk_ver_win_start = user_cfg->blk_ver_win_start;
		update = 1;
	}
	if (cur_cfg->blk_win_height != user_cfg->blk_win_height) {
		cur_cfg->blk_win_height = user_cfg->blk_win_height;
		update = 1;
	}
	if (cur_cfg->subsample_ver_inc != user_cfg->subsample_ver_inc) {
		cur_cfg->subsample_ver_inc = user_cfg->subsample_ver_inc;
		update = 1;
	}
	if (cur_cfg->subsample_hor_inc != user_cfg->subsample_hor_inc) {
		cur_cfg->subsample_hor_inc = user_cfg->subsample_hor_inc;
		update = 1;
	}

	if (update || !aewb->configured) {
		aewb->inc_config++;
		aewb->update = 1;
		cur_cfg->buf_size = isph3a_aewb_get_buf_size(cur_cfg);
	}
}

static long isph3a_aewb_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
			      void *arg)
{
	struct ispstat *stat = v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_PRIVATE_ISP_AEWB_CFG:
		return ispstat_config(stat, arg);
	case VIDIOC_PRIVATE_ISP_STAT_REQ:
		return ispstat_request_statistics(stat, arg);
	case VIDIOC_PRIVATE_ISP_STAT_EN: {
		unsigned long *en = arg;
		return ispstat_enable(stat, !!*en);
	}
	}

	return -ENOIOCTLCMD;
}

static const struct ispstat_ops isph3a_aewb_ops = {
	.validate_params = isph3a_aewb_validate_params,
	.set_params = isph3a_aewb_set_params,
	.setup_regs = isph3a_aewb_setup_regs,
};

static const struct ispstat_pcr_bits isph3a_aewb_pcr = {
	.base = OMAP3_ISP_IOMEM_H3A,
	.offset = ISPH3A_PCR,
	.enable = ISPH3A_PCR_AEW_EN,
	.busy = ISPH3A_PCR_BUSYAEAWB,
};

static const struct v4l2_subdev_core_ops isph3a_aewb_subdev_core_ops = {
	.ioctl = isph3a_aewb_ioctl,
	.subscribe_event = ispstat_subscribe_event,
	.unsubscribe_event = ispstat_unsubscribe_event,
};

static const struct v4l2_subdev_video_ops isph3a_aewb_subdev_video_ops = {
	.s_stream = ispstat_s_stream,
};

static const struct v4l2_subdev_ops isph3a_aewb_subdev_ops = {
	.core = &isph3a_aewb_subdev_core_ops,
	.video = &isph3a_aewb_subdev_video_ops,
};

/**
 * isph3a_aewb_init - Module Initialisation.
 **/
int isph3a_aewb_init(struct isp_device *isp)
{
	struct ispstat *aewb = &isp->isp_aewb;
	struct isph3a_aewb_config *aewb_cfg;

	aewb_cfg = kzalloc(sizeof(*aewb_cfg), GFP_KERNEL);
	if (!aewb_cfg)
		return -ENOMEM;

	memset(aewb, 0, sizeof(*aewb));
	aewb->ops = &isph3a_aewb_ops;
	aewb->pcr = &isph3a_aewb_pcr;
	aewb->priv = aewb_cfg;
	aewb->dma_ch = -1;
	aewb->event_type = V4L2_EVENT_OMAP3ISP_AEWB;
	aewb->isp = isp;

	return ispstat_init(aewb, "AEWB", &isph3a_aewb_subdev_ops);
}

/**
 * isph3a_aewb_cleanup - Module exit.
 **/
void isph3a_aewb_cleanup(struct isp_device *isp)
{
	kfree(isp->isp_aewb.priv);
	ispstat_free(&isp->isp_aewb);
}

