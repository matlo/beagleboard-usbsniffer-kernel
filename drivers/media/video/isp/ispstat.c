/*
 * ispstat.c
 *
 * STAT module for TI's OMAP3 Camera ISP
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

#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#include "isp.h"

#define IS_DMA_BUF(stat)	((stat)->dma_ch >= 0)
#define MAGIC_NUM		0x55
#define MAGIC_SIZE		32

static int ispstat_buf_check_magic(struct ispstat *stat,
				   struct ispstat_buffer *buf)
{
	const int size = buf->buf_size >= MAGIC_SIZE ?
					  MAGIC_SIZE : buf->buf_size;
	u8 *w;
	u8 *end;
	int ret = -EINVAL;

	/* Checking initial magic numbers. They shouldn't be here anymore. */
	for (w = buf->virt_addr, end = w + size; w < end; w++)
		if (likely(*w != MAGIC_NUM))
			ret = 0;

	if (ret) {
		dev_dbg(stat->isp->dev, "%s: beginning magic check does not "
					"match.\n", stat->subdev.name);
		return ret;
	}

	/* Checking magic numbers at the end. They must be still here. */
	for (w = buf->virt_addr + buf->buf_size, end = w + MAGIC_SIZE;
	     w < end; w++) {
		if (unlikely(*w != MAGIC_NUM)) {
			dev_dbg(stat->isp->dev, "%s: endding magic check does "
				"not match.\n", stat->subdev.name);
			return -EINVAL;
		}
	}

	return 0;
}

static void ispstat_buf_insert_magic(struct ispstat *stat,
				     struct ispstat_buffer *buf)
{
	/*
	 * Inserting MAGIC_NUM at the beginning and end of the buffer.
	 * buf->buf_size is set only after the buffer is queued. For now the
	 * right buf_size for the current configuration is pointed by
	 * stat->buf_size.
	 */
	BUG_ON(!buf);
	BUG_ON(!buf->virt_addr);
	if (stat->buf_size >= MAGIC_SIZE)
		memset(buf->virt_addr, MAGIC_NUM, MAGIC_SIZE);
	else
		memset(buf->virt_addr, MAGIC_NUM, stat->buf_size);
	memset(buf->virt_addr + stat->buf_size, MAGIC_NUM, MAGIC_SIZE);
}

static void ispstat_buf_clear(struct ispstat *stat)
{
	int i;

	for (i = 0; i < STAT_MAX_BUFS; i++)
		stat->buf[i].empty = 1;
}

static struct ispstat_buffer *__ispstat_buf_find(struct ispstat *stat,
						 int look_empty)
{
	struct ispstat_buffer *found = NULL;
	int i;

	for (i = 0; i < STAT_MAX_BUFS; i++) {
		struct ispstat_buffer *curr = &stat->buf[i];

		/*
		 * Don't select the buffer which is being copied to
		 * userspace or used by the module.
		 */
		if (curr == stat->locked_buf || curr == stat->active_buf)
			continue;

		/* Don't select uninitialised buffers if it's not required */
		if (!look_empty && curr->empty)
			continue;

		/* Pick uninitialised buffer over anything else if look_empty */
		if (curr->empty) {
			found = curr;
			break;
		}

		/* Choose the oldest buffer */
		if (!found ||
		    (s32)curr->frame_number - (s32)found->frame_number < 0)
			found = curr;
	}

	return found;
}

static inline struct ispstat_buffer *
ispstat_buf_find_oldest(struct ispstat *stat)
{
	return __ispstat_buf_find(stat, 0);
}

static inline struct ispstat_buffer *
ispstat_buf_find_oldest_or_empty(struct ispstat *stat)
{
	return __ispstat_buf_find(stat, 1);
}

static int ispstat_buf_queue(struct ispstat *stat)
{
	if (!stat->active_buf)
		return STAT_NO_BUF;

	do_gettimeofday(&stat->active_buf->ts);

	stat->active_buf->buf_size = stat->buf_size;
	if (ispstat_buf_check_magic(stat, stat->active_buf)) {
		dev_dbg(stat->isp->dev, "%s: data wasn't properly written.\n",
			stat->subdev.name);
		return STAT_NO_BUF;
	}
	stat->active_buf->config_counter = stat->config_counter;
	stat->active_buf->frame_number = stat->frame_number;
	stat->active_buf->empty = 0;
	stat->active_buf = NULL;
	stat->frame_number++;

	return STAT_BUF_DONE;
}

/* Get next free buffer to write the statistics to and mark it active. */
static void ispstat_buf_next(struct ispstat *stat)
{

	if (unlikely(stat->active_buf))
		/* Overwriting unused active buffer */
		dev_dbg(stat->isp->dev, "%s: new buffer requested without "
					"queuing active one.\n",
					stat->subdev.name);
	else
		stat->active_buf = ispstat_buf_find_oldest_or_empty(stat);
}

static void ispstat_buf_release(struct ispstat *stat)
{
	unsigned long flags;

	spin_lock_irqsave(&stat->isp->stat_lock, flags);
	stat->locked_buf = NULL;
	spin_unlock_irqrestore(&stat->isp->stat_lock, flags);
}

/* Get buffer to userspace. */
static struct ispstat_buffer *ispstat_buf_get(struct ispstat *stat,
					      struct ispstat_data *data)
{
	int rval = 0;
	unsigned long flags;
	struct ispstat_buffer *buf;

	spin_lock_irqsave(&stat->isp->stat_lock, flags);

	while (1) {
		buf = ispstat_buf_find_oldest(stat);
		if (!buf) {
			spin_unlock_irqrestore(&stat->isp->stat_lock, flags);
			dev_dbg(stat->isp->dev, "%s: cannot find a buffer.\n",
				stat->subdev.name);
			return ERR_PTR(-EBUSY);
		}
		if (ispstat_buf_check_magic(stat, buf)) {
			dev_dbg(stat->isp->dev, "%s: current buffer has "
				"corrupted data\n.", stat->subdev.name);
			/* Mark empty because it doesn't have valid data. */
			buf->empty = 1;
		} else {
			/* Buffer isn't corrupted. */
			break;
		}
	}

	stat->locked_buf = buf;

	spin_unlock_irqrestore(&stat->isp->stat_lock, flags);

	if (buf->buf_size > data->buf_size) {
		dev_dbg(stat->isp->dev, "%s: userspace's buffer size is "
					"not enough.\n", stat->subdev.name);
		return ERR_PTR(-EINVAL);
	}

	rval = copy_to_user(data->buf,
			    buf->virt_addr,
			    buf->buf_size);

	if (rval) {
		dev_info(stat->isp->dev,
			 "%s: failed copying %d bytes of stat data\n",
			 stat->subdev.name, rval);
		buf = ERR_PTR(-EFAULT);
		ispstat_buf_release(stat);
	}

	return buf;
}

static void ispstat_bufs_free(struct ispstat *stat)
{
	struct isp_device *isp = dev_get_drvdata(stat->isp->dev);
	int i;

	for (i = 0; i < STAT_MAX_BUFS; i++) {
		struct ispstat_buffer *buf = &stat->buf[i];

		if (!IS_DMA_BUF(stat)) {
			if (IS_ERR_OR_NULL((void *)buf->iommu_addr))
				continue;
			iommu_vfree(isp->iommu, buf->iommu_addr);
		} else {
			if (!buf->virt_addr)
				continue;
			dma_free_coherent(stat->isp->dev, stat->buf_alloc_size,
					  buf->virt_addr, buf->dma_addr);
		}
		buf->iommu_addr = 0;
		buf->dma_addr = 0;
		buf->virt_addr = NULL;
		buf->empty = 1;
	}

	dev_dbg(stat->isp->dev, "%s: all buffers were freed.\n",
		stat->subdev.name);

	stat->buf_alloc_size = 0;
	stat->active_buf = NULL;
}

static int ispstat_bufs_alloc_iommu(struct ispstat *stat, unsigned int size)
{
	struct isp_device *isp = dev_get_drvdata(stat->isp->dev);
	int i;

	stat->buf_alloc_size = size;

	for (i = 0; i < STAT_MAX_BUFS; i++) {
		struct ispstat_buffer *buf = &stat->buf[i];

		WARN_ON(buf->dma_addr);
		buf->iommu_addr = iommu_vmalloc(isp->iommu, 0, size,
						IOMMU_FLAG);
		if (IS_ERR((void *)buf->iommu_addr)) {
			dev_err(stat->isp->dev,
				 "%s: Can't acquire memory for "
				 "buffer %d\n", stat->subdev.name, i);
			ispstat_bufs_free(stat);
			return -ENOMEM;
		}
		buf->virt_addr = da_to_va(stat->isp->iommu,
					  (u32)buf->iommu_addr);
		buf->empty = 1;
		dev_dbg(stat->isp->dev, "%s: buffer[%d] allocated."
			"iommu_addr=0x%08lx virt_addr=0x%08lx",
			stat->subdev.name, i, buf->iommu_addr,
			(unsigned long)buf->virt_addr);
	}

	return 0;
}

static int ispstat_bufs_alloc_dma(struct ispstat *stat, unsigned int size)
{
	int i;

	stat->buf_alloc_size = size;

	for (i = 0; i < STAT_MAX_BUFS; i++) {
		struct ispstat_buffer *buf = &stat->buf[i];

		WARN_ON(buf->iommu_addr);
		buf->virt_addr = dma_alloc_coherent(stat->isp->dev, size,
					&buf->dma_addr, GFP_KERNEL | GFP_DMA);

		if (!buf->virt_addr || !buf->dma_addr) {
			dev_info(stat->isp->dev,
				 "%s: Can't acquire memory for "
				 "DMA buffer %d\n", stat->subdev.name, i);
			ispstat_bufs_free(stat);
			return -ENOMEM;
		}
		buf->empty = 1;

		dev_dbg(stat->isp->dev, "%s: buffer[%d] allocated."
			"dma_addr=0x%08lx virt_addr=0x%08lx\n",
			stat->subdev.name, i, (unsigned long)buf->dma_addr,
			(unsigned long)buf->virt_addr);
	}

	return 0;
}

static int ispstat_bufs_alloc(struct ispstat *stat, u32 size)
{
	unsigned long flags;

	spin_lock_irqsave(&stat->isp->stat_lock, flags);

	BUG_ON(stat->locked_buf != NULL);

	/* Are the old buffers big enough? */
	if (stat->buf_alloc_size >= size) {
		spin_unlock_irqrestore(&stat->isp->stat_lock, flags);
		return 0;
	}

	if (stat->state != ISPSTAT_DISABLED || stat->buf_processing) {
		dev_info(stat->isp->dev,
			 "%s: trying to allocate memory when busy\n",
			 stat->subdev.name);
		spin_unlock_irqrestore(&stat->isp->stat_lock, flags);
		return -EBUSY;
	}

	spin_unlock_irqrestore(&stat->isp->stat_lock, flags);

	ispstat_bufs_free(stat);

	if (IS_DMA_BUF(stat))
		return ispstat_bufs_alloc_dma(stat, size);
	else
		return ispstat_bufs_alloc_iommu(stat, size);
}

static void ispstat_queue_event(struct ispstat *stat, int err)
{
	struct video_device *vdev = &stat->subdev.devnode;
	struct v4l2_event event;
	struct ispstat_event_status *status = (void *)event.u.data;

	memset(&event, 0, sizeof(event));
	if (!err) {
		status->frame_number = stat->frame_number;
		status->config_counter = stat->config_counter;
	} else {
		status->buf_err = 1;
	}
	event.type = stat->event_type;
	v4l2_event_queue(vdev, &event);
}


/**
 * ispstat_request_statistics - Request statistics.
 * @data: Pointer to return statistics data.
 *
 * Returns 0 if successful.
 */
int ispstat_request_statistics(struct ispstat *stat,
			       struct ispstat_data *data)
{
	struct ispstat_buffer *buf;

	if (stat->state != ISPSTAT_ENABLED) {
		dev_dbg(stat->isp->dev, "%s: engine not enabled.\n",
			stat->subdev.name);
		return -EINVAL;
	}

	buf = ispstat_buf_get(stat, data);
	if (IS_ERR(buf))
		return PTR_ERR(buf);

	data->ts = buf->ts;
	data->config_counter = buf->config_counter;
	data->frame_number = buf->frame_number;
	data->buf_size = buf->buf_size;

	/*
	 * Deprecated. Number of new buffers is always equal to number of
	 * queued events without error flag. By setting it to 0, userspace
	 * won't try to request new buffer without receiving new event.
	 * This field must go away in future.
	 */
	data->new_bufs = 0;

	buf->empty = 1;
	ispstat_buf_release(stat);

	return 0;
}

/**
 * ispstat_config - Receives new statistic engine configuration.
 * @new_conf: Pointer to config structure.
 *
 * Returns 0 if successful, -EINVAL if new_conf pointer is NULL, -ENOMEM if
 * was unable to allocate memory for the buffer, or other errors if parameters
 * are invalid.
 */
int ispstat_config(struct ispstat *stat, void *new_conf)
{
	int ret;
	unsigned long irqflags;
	struct ispstat_generic_config *user_cfg = new_conf;
	u32 buf_size = user_cfg->buf_size;

	if (!new_conf) {
		dev_dbg(stat->isp->dev, "%s: configuration is NULL\n",
			stat->subdev.name);
		return -EINVAL;
	}

	mutex_lock(&stat->ioctl_lock);

	dev_dbg(stat->isp->dev, "%s: configuring module with buffer "
		"size=0x%08lx\n", stat->subdev.name, (unsigned long)buf_size);

	ret = stat->ops->validate_params(stat, new_conf);
	if (ret) {
		mutex_unlock(&stat->ioctl_lock);
		dev_dbg(stat->isp->dev, "%s: configuration values are "
					"invalid.\n", stat->subdev.name);
		return ret;
	}

	if (buf_size != user_cfg->buf_size)
		dev_dbg(stat->isp->dev, "%s: driver has corrected buffer size "
			"request to 0x%08lx\n", stat->subdev.name,
			(unsigned long)user_cfg->buf_size);

	/*
	 * Hack: H3A modules may need a doubled buffer size to avoid access
	 * to a invalid memory address after a SBL overflow.
	 * The buffer size is always PAGE_ALIGNED.
	 * Hack 2: MAGIC_SIZE is added to buf_size so a magic word can be
	 * inserted at the end to data integrity check purpose.
	 */
	if (stat == &stat->isp->isp_aewb || stat == &stat->isp->isp_af)
		buf_size = PAGE_ALIGN(user_cfg->buf_size * 2 + MAGIC_SIZE);
	else
		buf_size = PAGE_ALIGN(user_cfg->buf_size + MAGIC_SIZE);

	ret = ispstat_bufs_alloc(stat, buf_size);
	if (ret) {
		mutex_unlock(&stat->ioctl_lock);
		return ret;
	}

	spin_lock_irqsave(&stat->isp->stat_lock, irqflags);
	stat->ops->set_params(stat, new_conf);
	spin_unlock_irqrestore(&stat->isp->stat_lock, irqflags);

	/*
	 * Returning the right future config_counter for this setup, so
	 * userspace can *know* when it has been applied.
	 */
	user_cfg->config_counter = stat->config_counter + stat->inc_config;

	/* Module has a valid configuration. */
	stat->configured = 1;
	dev_dbg(stat->isp->dev, "%s: module has been successfully "
		"configured.\n", stat->subdev.name);

	mutex_unlock(&stat->ioctl_lock);

	return 0;
}

/**
 * ispstat_buf_process - Process statistic buffers.
 */
static int ispstat_buf_process(struct ispstat *stat)
{
	int ret = STAT_NO_BUF;

	if (likely(!stat->buf_err && stat->state == ISPSTAT_ENABLED)) {
		ret = ispstat_buf_queue(stat);
		ispstat_buf_next(stat);
	} else {
		stat->buf_err = 0;
	}

	return ret;
}

int ispstat_busy(struct ispstat *stat)
{
	return isp_reg_readl(stat->isp, stat->pcr->base, stat->pcr->offset)
		& stat->pcr->busy;
}

static void __ispstat_pcr_enable(struct ispstat *stat, u8 pcr_enable)
{
	u32 pcr = isp_reg_readl(stat->isp, stat->pcr->base,
				stat->pcr->offset);

	if (pcr_enable)
		pcr |= stat->pcr->enable;
	else
		pcr &= ~stat->pcr->enable;
	isp_reg_writel(stat->isp, pcr, stat->pcr->base, stat->pcr->offset);
}

/**
 * ispstat_pcr_enable - Disables/Enables statistic engines.
 * @pcr_enable: 0/1 - Disables/Enables the engine.
 *
 * Must be called from ISP driver only and not from a userspace request.
 */
void ispstat_pcr_enable(struct ispstat *stat, u8 pcr_enable)
{
	unsigned long irqflags;

	spin_lock_irqsave(&stat->isp->stat_lock, irqflags);

	if ((stat->state == ISPSTAT_DISABLING ||
	     stat->state == ISPSTAT_DISABLED) && pcr_enable) {
		/* Userspace has disabled the module. Aborting. */
		spin_unlock_irqrestore(&stat->isp->stat_lock, irqflags);
		return;
	}

	__ispstat_pcr_enable(stat, pcr_enable);
	if (stat->state == ISPSTAT_DISABLING && !pcr_enable)
		stat->state = ISPSTAT_DISABLED;
	else if (stat->state == ISPSTAT_ENABLING && pcr_enable)
		stat->state = ISPSTAT_ENABLED;

	spin_unlock_irqrestore(&stat->isp->stat_lock, irqflags);
}

void ispstat_suspend(struct ispstat *stat)
{
	unsigned long flags;

	spin_lock_irqsave(&stat->isp->stat_lock, flags);

	if (stat->state != ISPSTAT_DISABLED)
		__ispstat_pcr_enable(stat, 0);

	spin_unlock_irqrestore(&stat->isp->stat_lock, flags);
}

void ispstat_resume(struct ispstat *stat)
{
	unsigned long flags;

	spin_lock_irqsave(&stat->isp->stat_lock, flags);

	if (stat->state == ISPSTAT_ENABLED) {
		stat->update = 1;
		stat->ops->setup_regs(stat);
		__ispstat_pcr_enable(stat, 1);
	}

	spin_unlock_irqrestore(&stat->isp->stat_lock, flags);
}

static void ispstat_try_enable(struct ispstat *stat)
{
	unsigned long irqflags;

	if (stat->priv == NULL)
		/* driver wasn't initialised */
		return;

	spin_lock_irqsave(&stat->isp->stat_lock, irqflags);
	if (stat->state == ISPSTAT_ENABLING && !stat->buf_processing &&
	    stat->buf_alloc_size) {
		/*
		 * Userspace's requested to enable the engine but it wasn't yet.
		 * Let's do that now.
		 */
		stat->update = 1;
		ispstat_buf_next(stat);
		stat->ops->setup_regs(stat);
		ispstat_buf_insert_magic(stat, stat->active_buf);
		spin_unlock_irqrestore(&stat->isp->stat_lock, irqflags);
		ispstat_pcr_enable(stat, 1);
		dev_dbg(stat->isp->dev, "%s: module is enabled.\n",
			stat->subdev.name);
	} else {
		spin_unlock_irqrestore(&stat->isp->stat_lock, irqflags);
	}
}

/**
 * ispstat_enable - Disables/Enables statistic engine as soon as it's possible.
 * @enable: 0/1 - Disables/Enables the engine.
 *
 * Client should configure all the module registers before this.
 * This function can be called from a userspace request.
 */
int ispstat_enable(struct ispstat *stat, u8 enable)
{
	unsigned long irqflags;

	dev_dbg(stat->isp->dev, "%s: user wants to %s module.\n",
		stat->subdev.name, enable ? "enable" : "disable");

	/* Prevent enabling while configuring */
	mutex_lock(&stat->ioctl_lock);

	spin_lock_irqsave(&stat->isp->stat_lock, irqflags);

	if (!stat->configured && enable) {
		spin_unlock_irqrestore(&stat->isp->stat_lock, irqflags);
		mutex_unlock(&stat->ioctl_lock);
		dev_dbg(stat->isp->dev, "%s: cannot enable module as it's "
			"never been successfully configured so far.\n",
			stat->subdev.name);
		return -EINVAL;
	}

	if (enable) {
		if (stat->state == ISPSTAT_DISABLING)
			/* Previous disabling request wasn't done yet */
			stat->state = ISPSTAT_ENABLED;
		else if (stat->state == ISPSTAT_DISABLED)
			/* Module is now being enabled */
			stat->state = ISPSTAT_ENABLING;
	} else {
		if (stat->state == ISPSTAT_ENABLING) {
			/* Previous enabling request wasn't done yet */
			stat->state = ISPSTAT_DISABLED;
		} else if (stat->state == ISPSTAT_ENABLED) {
			/* Module is now being disabled */
			stat->state = ISPSTAT_DISABLING;
			ispstat_buf_clear(stat);
		}
	}

	spin_unlock_irqrestore(&stat->isp->stat_lock, irqflags);
	mutex_unlock(&stat->ioctl_lock);

	return 0;
}

int ispstat_s_stream(struct v4l2_subdev *subdev, int enable)
{
	struct ispstat *stat = v4l2_get_subdevdata(subdev);

	if (enable) {
		/*
		 * Only set enable PCR bit if the module was previously
		 * enabled through ioct.
		 */
		ispstat_try_enable(stat);
	} else {
		/* Disable PCR bit and config enable field */
		ispstat_enable(stat, 0);
		ispstat_pcr_enable(stat, 0);
		dev_dbg(stat->isp->dev, "%s: module is being disabled\n",
			stat->subdev.name);
	}

	return 0;
}

/**
 * __ispstat_isr - Interrupt handler for statistic drivers
 */
static void __ispstat_isr(struct ispstat *stat, int from_dma)
{
	int ret = STAT_BUF_DONE;
	unsigned long irqflags;

	ispstat_pcr_enable(stat, 0);
	/* If it's busy we can't process this buffer anymore */
	if (!ispstat_busy(stat)) {
		if (!from_dma && stat->ops->buf_process &&
		    !stat->buf_processing) {
			/* Module still need to copy data to buffer. */
			ret = stat->ops->buf_process(stat);
		}
		stat->buf_processing = 1;
		if (ret == STAT_BUF_WAITING_DMA)
			/* Buffer is not ready yet */
			return;

		spin_lock_irqsave(&stat->isp->stat_lock, irqflags);
		if (ret == STAT_BUF_DONE)
			/* Buffer is ready to be processed */
			ret = ispstat_buf_process(stat);
		stat->ops->setup_regs(stat);
		ispstat_buf_insert_magic(stat, stat->active_buf);
		spin_unlock_irqrestore(&stat->isp->stat_lock, irqflags);

		/*
		 * Hack: H3A modules may access invalid memory address or send
		 * corrupted data to userspace if more than 1 SBL overflow
		 * happens in a row without re-writing its buffer's start memory
		 * address in the meantime. Such situation is avoided if the
		 * module is not immediately re-enabled when the ISR misses the
		 * timing to process the buffer and to setup the registers.
		 * Because of that, pcr_enable(1) was moved to inside this 'if'
		 * block. But the next interruption will still happen as during
		 * pcr_enable(0) the module was busy.
		 */
		ispstat_pcr_enable(stat, 1);
	} else {
		/*
		 * If a SBL overflow occurs and the H3A driver misses the timing
		 * to process the buffer, stat->buf_err is set and won't be
		 * cleared now. So the next buffer will be correctly ignored.
		 * It's necessary due to a hw issue which makes the next H3A
		 * buffer to start from the memory address where the previous
		 * one stopped, instead of start where it was configured to.
		 * Do not "stat->buf_err = 0" here.
		 */

		if (stat->ops->buf_process)
			/*
			 * Driver may need to erase current data prior to
			 * process a new buffer. If it misses the timing, the
			 * next buffer might be wrong. So should be ignored.
			 * It happens only for Histogram.
			 */
			stat->buf_err = 1;

		ret = STAT_NO_BUF;
		dev_dbg(stat->isp->dev, "%s: cannot process buffer, "
					"device is busy.\n", stat->subdev.name);
	}
	stat->buf_processing = 0;
	ispstat_queue_event(stat, ret != STAT_BUF_DONE);
}

void ispstat_isr(struct ispstat *stat)
{
	__ispstat_isr(stat, 0);
}

void ispstat_dma_isr(struct ispstat *stat)
{
	__ispstat_isr(stat, 1);
}

static int ispstat_init_entities(struct ispstat *stat, const char *name,
				 const struct v4l2_subdev_ops *sd_ops)
{
	struct v4l2_subdev *subdev = &stat->subdev;
	struct media_entity *me = &subdev->entity;

	v4l2_subdev_init(subdev, sd_ops);
	snprintf(subdev->name, V4L2_SUBDEV_NAME_SIZE, "OMAP3 ISP %s", name);
	subdev->grp_id = 1 << 16;	/* group ID for isp subdevs */
	subdev->flags |= V4L2_SUBDEV_USES_EVENTS;
	subdev->nevents = STAT_NEVENTS;
	v4l2_set_subdevdata(subdev, stat);

	stat->pad.type = MEDIA_PAD_TYPE_INPUT;
	me->ops = NULL;

	return media_entity_init(me, 1, &stat->pad, 0);
}

int ispstat_subscribe_event(struct v4l2_subdev *subdev, struct v4l2_fh *fh,
			    struct v4l2_event_subscription *sub)
{
	struct ispstat *stat = v4l2_get_subdevdata(subdev);

	if (sub->type != stat->event_type)
		return -EINVAL;

	return v4l2_event_subscribe(fh, sub);
}

int ispstat_unsubscribe_event(struct v4l2_subdev *subdev, struct v4l2_fh *fh,
			      struct v4l2_event_subscription *sub)
{
	return v4l2_event_unsubscribe(fh, sub);
}

void ispstat_unregister_entities(struct ispstat *stat)
{
	media_entity_cleanup(&stat->subdev.entity);
	v4l2_device_unregister_subdev(&stat->subdev);
}

int ispstat_register_entities(struct ispstat *stat, struct v4l2_device *vdev)
{
	return v4l2_device_register_subdev(vdev, &stat->subdev);
}

int ispstat_init(struct ispstat *stat, const char *name,
		 const struct v4l2_subdev_ops *sd_ops)
{
	stat->buf = kcalloc(STAT_MAX_BUFS, sizeof(*stat->buf), GFP_KERNEL);
	if (!stat->buf)
		return -ENOMEM;
	ispstat_buf_clear(stat);
	mutex_init(&stat->ioctl_lock);

	return ispstat_init_entities(stat, name, sd_ops);
}

void ispstat_free(struct ispstat *stat)
{
	ispstat_bufs_free(stat);
	kfree(stat->buf);
}

