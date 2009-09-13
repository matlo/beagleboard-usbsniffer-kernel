/*
 * ispstat.c
 *
 * STAT module for TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contributors:
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

#include <linux/dma-mapping.h>
#include <linux/uaccess.h>

#include "isp.h"

/* Get next free buffer to write the statistics to and mark it active. */
struct ispstat_buffer *ispstat_buf_next(struct ispstat *stat)
{
	unsigned long flags;
	struct ispstat_buffer *found = NULL;
	int i;

	if (stat->active_buf)
		do_gettimeofday(&stat->active_buf->ts);

	spin_lock_irqsave(&stat->lock, flags);

	if (stat->active_buf) {
		stat->active_buf->config_counter = stat->config_counter;
		stat->active_buf->frame_number = stat->frame_number;
	}

	for (i = 0; i < stat->nbufs; i++) {
		struct ispstat_buffer *curr = &stat->buf[i];

		/*
		 * Don't select the buffer which is being copied to
		 * userspace.
		 */
		if (curr == stat->locked_buf)
			continue;

		if (!found
		    || (curr->frame_number > found->frame_number
			&& (curr->frame_number - found->frame_number
			    > stat->max_frame / 2))
		    || (curr->frame_number < found->frame_number
			&& (found->frame_number - curr->frame_number
			    < stat->max_frame / 2))) {
			found = curr;
		}
	}

	stat->active_buf = found;

	stat->frame_number++;
	if (stat->frame_number == stat->max_frame)
		stat->frame_number = 0;

	spin_unlock_irqrestore(&stat->lock, flags);

	return found;
}

/* Get buffer to userspace. */
static struct ispstat_buffer *ispstat_buf_find(
	struct ispstat *stat, u32 frame_number)
{
	struct ispstat_buffer *latest = NULL;
	int i;

	for (i = 0; i < stat->nbufs; i++) {
		struct ispstat_buffer *curr = &stat->buf[i];

		/* We cannot deal with the active buffer. */
		if (curr == stat->active_buf)
			continue;

		/* Don't take uninitialised buffers. */
		if (curr->frame_number == stat->max_frame)
			continue;

		/* Found correct number. */
		if (curr->frame_number == frame_number) {
			latest = curr;
			break;
		}

		/* Select first buffer or a better one. */
		if (!latest
		    || (curr->frame_number < latest->frame_number
			&& (latest->frame_number - curr->frame_number
			    > stat->max_frame / 2))
		    || (curr->frame_number > latest->frame_number
			&& (curr->frame_number - latest->frame_number
			    < stat->max_frame / 2)))
			latest = curr;
	}

	return latest;
}

/**
 * ispstat_stats_available - Check for stats available of specified frame.
 * @aewbdata: Pointer to return AE AWB statistics data
 *
 * Returns 0 if successful, or -1 if statistics are unavailable.
 **/
struct ispstat_buffer *ispstat_buf_get(struct ispstat *stat,
				       void __user *ptr,
				       unsigned int frame_number)
{
	int rval = 0;
	unsigned long flags;
	struct ispstat_buffer *buf;

	spin_lock_irqsave(&stat->lock, flags);

	buf = ispstat_buf_find(stat, frame_number);
	if (!buf) {
		spin_unlock_irqrestore(&stat->lock, flags);
		return ERR_PTR(-EBUSY);
	}

	stat->locked_buf = buf;

	spin_unlock_irqrestore(&stat->lock, flags);

	rval = copy_to_user((void *)ptr,
			    buf->virt_addr,
			    stat->buf_size);

	if (rval) {
		dev_info(stat->dev,
			 "failed copying %d bytes of stat data\n", rval);
		buf = ERR_PTR(-EFAULT);
		ispstat_buf_release(stat);
	}

	return buf;
}

void ispstat_buf_release(struct ispstat *stat)
{
	unsigned long flags;

	spin_lock_irqsave(&stat->lock, flags);
	stat->locked_buf = NULL;
	spin_unlock_irqrestore(&stat->lock, flags);
}

void ispstat_bufs_free(struct ispstat *stat)
{
	struct isp_device *isp = dev_get_drvdata(stat->dev);
	int i;

	for (i = 0; i < stat->nbufs; i++) {
		struct ispstat_buffer *buf = &stat->buf[i];

		if (!buf->iommu_addr)
			continue;

		iommu_vfree(isp->iommu, buf->iommu_addr);
		buf->iommu_addr = 0;
	}

	stat->buf_alloc_size = 0;
}

int ispstat_bufs_alloc(struct ispstat *stat,
		       unsigned int size)
{
	struct isp_device *isp = dev_get_drvdata(stat->dev);
	unsigned long flags;
	int i;

	spin_lock_irqsave(&stat->lock, flags);

	BUG_ON(stat->locked_buf != NULL);

	/* Are the old buffers big enough? */
	if (stat->buf_alloc_size >= size) {
		for (i = 0; i < stat->nbufs; i++)
			stat->buf[i].frame_number = stat->max_frame;
		spin_unlock_irqrestore(&stat->lock, flags);
		goto out;
	}

	if (isp->running != ISP_STOPPED) {
		dev_info(stat->dev, "stat: trying to configure when busy\n");
		spin_unlock_irqrestore(&stat->lock, flags);
		return -EBUSY;
	}

	spin_unlock_irqrestore(&stat->lock, flags);

	ispstat_bufs_free(stat);

	for (i = 0; i < stat->nbufs; i++) {
		struct ispstat_buffer *buf = &stat->buf[i];

		buf->iommu_addr = iommu_vmalloc(isp->iommu, 0, size,
						IOMMU_FLAG);
		if (buf->iommu_addr == 0) {
			dev_info(stat->dev, "stat: Can't acquire memory for "
				 "buffer %d\n", i);
			ispstat_bufs_free(stat);
			return -ENOMEM;
		}
		buf->virt_addr = da_to_va(isp->iommu, (u32)buf->iommu_addr);
		buf->frame_number = stat->max_frame;
	}

	stat->buf_alloc_size = size;

out:
	stat->buf_size = size;
	stat->active_buf = NULL;

	return 0;
}

int ispstat_init(struct device *dev, struct ispstat *stat,
		 unsigned int nbufs, unsigned int max_frame)
{
	BUG_ON(nbufs < 2);
	BUG_ON(max_frame < 2);
	BUG_ON(nbufs >= max_frame);

	memset(stat, 0, sizeof(*stat));

	stat->buf = kcalloc(nbufs, sizeof(*stat->buf), GFP_KERNEL);
	if (!stat->buf)
		return -ENOMEM;

	spin_lock_init(&stat->lock);
	stat->nbufs = nbufs;
	stat->dev = dev;
	stat->max_frame = max_frame;

	return 0;
}

void ispstat_free(struct ispstat *stat)
{
	ispstat_bufs_free(stat);
	kfree(stat->buf);
}
