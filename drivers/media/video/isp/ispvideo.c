/*
 * ispvideo.c - ISP generic video node
 *
 * Copyright (C) 2009-2010 Nokia.
 *
 * Contributors:
 * 	Laurent Pinchart <laurent.pinchart@ideasonboard.com>
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
#include <linux/mm.h>
#include <linux/pagemap.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <plat/iommu.h>
#include <plat/iovmm.h>
#include <plat/omap-pm.h>

#include "ispvideo.h"
#include "isp.h"


/* -----------------------------------------------------------------------------
 * Helper functions
 */

static struct v4l2_subdev *
isp_video_remote_subdev(struct isp_video *video, u32 *pad)
{
	struct media_entity_pad *remote;

	remote = media_entity_remote_pad(&video->pad);

	if (remote == NULL || remote->entity->type != MEDIA_ENTITY_TYPE_SUBDEV)
		return NULL;

	if (pad)
		*pad = remote->index;

	return media_entity_to_v4l2_subdev(remote->entity);

}

/* Return a pointer to the ISP video instance at the far end of the pipeline. */
static struct isp_video *
isp_video_far_end(struct isp_video *video)
{
	struct media_entity_graph graph;
	struct media_entity *entity = &video->video.entity;
	struct media_device *mdev = entity->parent;
	struct video_device *vdev = NULL;

	mutex_lock(&mdev->graph_mutex);
	media_entity_graph_walk_start(&graph, entity);

	while ((entity = media_entity_graph_walk_next(&graph))) {
		if (entity == &video->video.entity)
			continue;

		if (entity->type == MEDIA_ENTITY_TYPE_NODE) {
			vdev = media_entity_to_video_device(entity);
			break;
		}
	}

	mutex_unlock(&mdev->graph_mutex);
	return vdev ? to_isp_video(vdev) : NULL;
}

static int
__isp_video_get_format(struct isp_video *video, struct v4l2_format *format)
{
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = isp_video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EINVAL;

	mutex_lock(&video->mutex);
	ret = v4l2_subdev_call(subdev, pad, get_fmt, pad, &fmt,
			       V4L2_SUBDEV_FORMAT_ACTIVE);
	if (ret == -ENOIOCTLCMD)
		ret = -EINVAL;

	mutex_unlock(&video->mutex);

	if (ret)
		return ret;

	format->type = video->type;
	isp_video_mbus_to_pix(video, &fmt, &format->fmt.pix);
	return 0;
}

static int
isp_video_check_format(struct isp_video *video, struct isp_video_fh *vfh)
{
	struct v4l2_format format;
	int ret;

	ret = __isp_video_get_format(video, &format);
	if (ret < 0)
		return ret;

	if (vfh->format.fmt.pix.pixelformat != format.fmt.pix.pixelformat ||
	    vfh->format.fmt.pix.height != format.fmt.pix.height ||
	    vfh->format.fmt.pix.width != format.fmt.pix.width ||
	    vfh->format.fmt.pix.bytesperline != format.fmt.pix.bytesperline ||
	    vfh->format.fmt.pix.sizeimage != format.fmt.pix.sizeimage)
		return -EINVAL;

	return 0;
}

void isp_video_mbus_to_pix(const struct isp_video *video,
			   const struct v4l2_mbus_framefmt *mbus,
			   struct v4l2_pix_format *pix)
{
	memset(pix, 0, sizeof(*pix));
	pix->width = mbus->width;
	pix->height = mbus->height;

	switch (mbus->code) {
	case V4L2_MBUS_FMT_SGRBG10_1X10:
		pix->pixelformat = V4L2_PIX_FMT_SGRBG10;
		pix->bytesperline = pix->width * 2;
		break;
	case V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8:
		pix->pixelformat = V4L2_PIX_FMT_SGRBG10DPCM8;
		pix->bytesperline = pix->width;
		break;
	case V4L2_MBUS_FMT_YUYV16_1X16:
		pix->pixelformat = V4L2_PIX_FMT_YUYV;
		pix->bytesperline = pix->width * 2;
		break;
	case V4L2_MBUS_FMT_UYVY16_1X16:
	default:
		pix->pixelformat = V4L2_PIX_FMT_UYVY;
		pix->bytesperline = pix->width * 2;
		break;
	}

	if (video->alignment)
		pix->bytesperline = ALIGN(pix->bytesperline, video->alignment);

	pix->sizeimage = pix->bytesperline * pix->height;
	pix->colorspace = mbus->colorspace;
	pix->field = mbus->field;
}
EXPORT_SYMBOL_GPL(isp_video_mbus_to_pix);

void isp_video_pix_to_mbus(const struct v4l2_pix_format *pix,
			   struct v4l2_mbus_framefmt *mbus)
{
	memset(mbus, 0, sizeof(*mbus));
	mbus->width = pix->width;
	mbus->height = pix->height;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_SGRBG10:
		mbus->code = V4L2_MBUS_FMT_SGRBG10_1X10;
		break;
	case V4L2_PIX_FMT_SGRBG10DPCM8:
		mbus->code = V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8;
		break;
	case V4L2_PIX_FMT_YUYV:
		mbus->code = V4L2_MBUS_FMT_YUYV16_1X16;
		break;
	case V4L2_PIX_FMT_UYVY:
	default:
		mbus->code = V4L2_MBUS_FMT_UYVY16_1X16;
		break;
	}

	mbus->colorspace = pix->colorspace;
	mbus->field = pix->field;
}
EXPORT_SYMBOL_GPL(isp_video_pix_to_mbus);

/* -----------------------------------------------------------------------------
 * IOMMU management
 */

#define IOMMU_FLAG	(IOVMF_ENDIAN_LITTLE | IOVMF_ELSZ_8)

static struct scatterlist *ispmmu_sg_alloc(unsigned int nents, gfp_t gfp_mask)
{
	return vmalloc(nents * sizeof(struct scatterlist));
}

static void ispmmu_sg_free(struct scatterlist *sg, unsigned int nents)
{
	vfree(sg);
}

/**
 * ispmmu_vmap - Wrapper for Virtual memory mapping of a scatter gather list
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @sglist: Pointer to source Scatter gather list to allocate.
 * @sglen: Number of elements of the scatter-gatter list.
 *
 * Returns a resulting mapped device address by the ISP MMU, or -ENOMEM if
 * we ran out of memory.
 **/
static dma_addr_t
ispmmu_vmap(struct isp_device *isp, const struct scatterlist *sglist, int sglen)
{
	struct scatterlist *src = (struct scatterlist *)sglist;
	struct scatterlist *sg;
	struct sg_table *sgt;
	unsigned int i;
	int ret;
	u32 da;

	/*
	 * convert isp sglist to iommu sgt
	 * FIXME: should be fixed in the upper layer?
	 */
	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (sgt == NULL)
		return -ENOMEM;

	ret = __sg_alloc_table(sgt, sglen, -1, GFP_KERNEL, ispmmu_sg_alloc);
	if (ret)
		goto error;

	for_each_sg(sgt->sgl, sg, sgt->nents, i)
		sg_set_buf(sg, phys_to_virt(sg_dma_address(src + i)),
			   sg_dma_len(src + i));

	da = iommu_vmap(isp->iommu, 0, sgt, IOMMU_FLAG);
	if (IS_ERR_VALUE(da)) {
		ret = da;
		goto error;
	}

	return (dma_addr_t)da;

error:
	__sg_free_table(sgt, -1, ispmmu_sg_free);
	kfree(sgt);
	return ret;
}

/**
 * ispmmu_vunmap - Unmap a device address from the ISP MMU
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @da: Device address generated from a ispmmu_vmap call.
 **/
static void ispmmu_vunmap(struct isp_device *isp, dma_addr_t da)
{
	struct sg_table *sgt;

	sgt = iommu_vunmap(isp->iommu, (u32)da);
	if (sgt == NULL)
		return;

	__sg_free_table(sgt, -1, ispmmu_sg_free);
	kfree(sgt);
}

/* -----------------------------------------------------------------------------
 * Video queue operations
 */

static void isp_video_queue_prepare(struct isp_video_queue *queue,
				    unsigned int *nbuffers, unsigned int *size)
{
	struct isp_video_fh *vfh =
		container_of(queue, struct isp_video_fh, queue);
	struct isp_video *video = vfh->video;

	*size = vfh->format.fmt.pix.sizeimage;
	if (*size == 0)
		return;

	*nbuffers = min(*nbuffers, video->capture_mem / PAGE_ALIGN(*size));
}

static void isp_video_buffer_cleanup(struct isp_video_buffer *buf)
{
	struct isp_video_fh *vfh = isp_video_queue_to_isp_video_fh(buf->queue);
	struct isp_buffer *buffer = to_isp_buffer(buf);
	struct isp_video *video = vfh->video;

	if (buffer->isp_addr) {
		ispmmu_vunmap(video->isp, buffer->isp_addr);
		buffer->isp_addr = 0;
	}
}

static int isp_video_buffer_prepare(struct isp_video_buffer *buf)
{
	struct isp_video_fh *vfh = isp_video_queue_to_isp_video_fh(buf->queue);
	struct isp_buffer *buffer = to_isp_buffer(buf);
	struct isp_video *video = vfh->video;
	unsigned long addr;

	addr = ispmmu_vmap(video->isp, buf->sglist, buf->sglen);
	if (IS_ERR_VALUE(addr))
		return -EIO;

	if (!IS_ALIGNED(addr, 32)) {
		dev_dbg(video->isp->dev, "Buffer address must be "
			"aligned to 32 bytes boundary.\n");
		ispmmu_vunmap(video->isp, buffer->isp_addr);
		return -EINVAL;
	}

	buf->vbuf.bytesused = vfh->format.fmt.pix.sizeimage;
	buffer->isp_addr = addr;
	return 0;
}

/*
 * isp_video_buffer_queue - Add buffer to streaming queue
 * @buf: Video buffer
 *
 * In memory-to-memory mode, start streaming on the pipeline if buffers are
 * queued on both the input and the output, if the pipeline isn't already busy.
 * If the pipeline is busy, it will be restarted in the output module interrupt
 * handler.
 */
static void isp_video_buffer_queue(struct isp_video_buffer *buf)
{
	struct isp_video_fh *vfh = isp_video_queue_to_isp_video_fh(buf->queue);
	struct isp_buffer *buffer = to_isp_buffer(buf);
	struct isp_video *video = vfh->video;
	enum isp_pipeline_state state;
	unsigned long flags;
	unsigned int empty;
	unsigned int start;

	empty = list_empty(&video->dmaqueue);
	list_add_tail(&buffer->buffer.irqlist, &video->dmaqueue);

	if (empty) {
		if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			state = ISP_PIPELINE_QUEUE_OUTPUT;
		else
			state = ISP_PIPELINE_QUEUE_INPUT;

		spin_lock_irqsave(&video->pipe->lock, flags);
		video->pipe->state |= state;
		video->ops->queue(video, buffer);

		start = isp_pipeline_ready(video->pipe);
		if (start)
			video->pipe->state |= ISP_PIPELINE_STREAM;
		spin_unlock_irqrestore(&video->pipe->lock, flags);

		if (start)
			isp_pipeline_set_stream(video->isp, video->pipe->output,
						ISP_PIPELINE_STREAM_SINGLESHOT);
	}
}

static const struct isp_video_queue_operations isp_video_queue_ops = {
	.queue_prepare = &isp_video_queue_prepare,
	.buffer_prepare = &isp_video_buffer_prepare,
	.buffer_queue = &isp_video_buffer_queue,
	.buffer_cleanup = &isp_video_buffer_cleanup,
};

/*
 * isp_video_buffer_next - Complete the current buffer and return the next one
 * @video: ISP video object
 * @error: Whether an error occured during capture
 *
 * Remove the current video buffer from the DMA queue and fill its timestamp,
 * field count and state fields before waking up its completion handler.
 *
 * The buffer state is set to VIDEOBUF_DONE if no error occured (@error is 0)
 * or VIDEOBUF_ERROR otherwise (@error is non-zero).
 *
 * The DMA queue is expected to contain at least one buffer.
 *
 * Return a pointer to the next buffer in the DMA queue, or NULL if the queue is
 * empty.
 */
struct isp_buffer *isp_video_buffer_next(struct isp_video *video,
					 unsigned int error)
{
	struct isp_video_queue *queue = video->queue;
	enum isp_pipeline_state state;
	struct isp_video_buffer *buf;
	unsigned long flags;
	struct timespec ts;

	spin_lock_irqsave(&queue->irqlock, flags);
	BUG_ON(list_empty(&video->dmaqueue));
	buf = list_first_entry(&video->dmaqueue, struct isp_video_buffer,
			       irqlist);
	list_del(&buf->irqlist);
	spin_unlock_irqrestore(&queue->irqlock, flags);

	ktime_get_ts(&ts);
	buf->vbuf.timestamp.tv_sec = ts.tv_sec;
	buf->vbuf.timestamp.tv_usec = ts.tv_nsec / NSEC_PER_USEC;

	buf->vbuf.sequence = atomic_inc_return(&video->sequence);
	buf->state = error ? ISP_BUF_STATE_ERROR : ISP_BUF_STATE_DONE;

	wake_up(&buf->wait);

	if (list_empty(&video->dmaqueue)) {
		if (queue->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
			state = ISP_PIPELINE_QUEUE_OUTPUT
			      | ISP_PIPELINE_STREAM;
		else
			state = ISP_PIPELINE_QUEUE_INPUT
			      | ISP_PIPELINE_STREAM;

		spin_lock_irqsave(&video->pipe->lock, flags);
		video->pipe->state &= ~state;
		spin_unlock_irqrestore(&video->pipe->lock, flags);
		return NULL;
	}

	if (queue->type == V4L2_BUF_TYPE_VIDEO_CAPTURE &&
	    video->pipe->input != NULL) {
		spin_lock_irqsave(&video->pipe->lock, flags);
		video->pipe->state &= ~ISP_PIPELINE_STREAM;
		spin_unlock_irqrestore(&video->pipe->lock, flags);
	}

	buf = list_first_entry(&video->dmaqueue, struct isp_video_buffer,
			       irqlist);
	buf->state = ISP_BUF_STATE_ACTIVE;
	return to_isp_buffer(buf);
}

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int
isp_video_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct isp_video *video = video_drvdata(file);

	strlcpy(cap->driver, ISP_VIDEO_DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->card, video->video.name, sizeof(cap->card));
	strlcpy(cap->bus_info, "media", sizeof(cap->bus_info));
	cap->version = ISP_VIDEO_DRIVER_VERSION;

	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	else
		cap->capabilities = V4L2_CAP_VIDEO_OUTPUT | V4L2_CAP_STREAMING;

	return 0;
}

static int
isp_video_enum_formats(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct isp_video *video = video_drvdata(file);
	const struct v4l2_ioctl_ops *ops = video->ioctl_ops;

	if (ops == NULL || ops->vidioc_enum_fmt_vid_cap == NULL)
		return -EINVAL;

	return ops->vidioc_enum_fmt_vid_cap(file, fh, f);
}

static int
isp_video_enum_framesizes(struct file *file, void *fh,
			  struct v4l2_frmsizeenum *fsize)
{
	struct isp_video *video = video_drvdata(file);
	const struct v4l2_ioctl_ops *ops = video->ioctl_ops;

	if (ops == NULL || ops->vidioc_enum_framesizes == NULL)
		return -EINVAL;

	return ops->vidioc_enum_framesizes(file, fh, fsize);
}

static int
isp_video_enum_frameintervals(struct file *file, void *fh,
			      struct v4l2_frmivalenum *fival)
{
	struct isp_video *video = video_drvdata(file);
	const struct v4l2_ioctl_ops *ops = video->ioctl_ops;

	if (ops == NULL || ops->vidioc_enum_frameintervals == NULL)
		return -EINVAL;

	return ops->vidioc_enum_frameintervals(file, fh, fival);
}

static int
isp_video_get_format(struct file *file, void *fh, struct v4l2_format *format)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);

	if (format->type != video->type)
		return -EINVAL;

	mutex_lock(&video->mutex);
	*format = vfh->format;
	mutex_unlock(&video->mutex);

	return 0;
}

static int
isp_video_set_format(struct file *file, void *fh, struct v4l2_format *format)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);
	const struct v4l2_ioctl_ops *ops = video->ioctl_ops;
	struct v4l2_mbus_framefmt fmt;
	int ret = 0;

	if (format->type != video->type)
		return -EINVAL;

	mutex_lock(&video->mutex);

	/* Fill the bytesperline and sizeimage fields by converting to media bus
	 * format and back to pixel format.
	 */
	isp_video_pix_to_mbus(&format->fmt.pix, &fmt);
	isp_video_mbus_to_pix(video, &fmt, &format->fmt.pix);

	/* For legacy video nodes, set the format directly. New video nodes just
	 * store the format in the file handle and verify it at stream on time.
	 */
	if (ops != NULL && ops->vidioc_s_fmt_vid_cap != NULL) {
		ret = ops->vidioc_s_fmt_vid_cap(file, fh, format);
		if (ret < 0)
			goto done;
	}

	vfh->format = *format;

done:
	mutex_unlock(&video->mutex);
	return ret;
}

static int
isp_video_try_format(struct file *file, void *fh, struct v4l2_format *format)
{
	struct isp_video *video = video_drvdata(file);
	const struct v4l2_ioctl_ops *ops = video->ioctl_ops;
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	if (format->type != video->type)
		return -EINVAL;

	/* Try the legacy ioctl operation if available or the subdev pad
	 * operation otherwise.
	 */
	if (ops != NULL && ops->vidioc_try_fmt_vid_cap != NULL)
		return ops->vidioc_try_fmt_vid_cap(file, fh, format);

	subdev = isp_video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EINVAL;

	isp_video_pix_to_mbus(&format->fmt.pix, &fmt);

	ret = v4l2_subdev_call(subdev, pad, set_fmt, pad, &fmt,
			       V4L2_SUBDEV_FORMAT_PROBE);
	if (ret)
		return ret == -ENOIOCTLCMD ? -EINVAL : ret;

	isp_video_mbus_to_pix(video, &fmt, &format->fmt.pix);
	return 0;
}

static int
isp_video_query_control(struct file *file, void *fh,
			struct v4l2_queryctrl *query)
{
	struct isp_video *video = video_drvdata(file);
	const struct v4l2_ioctl_ops *ops = video->ioctl_ops;

	if (ops == NULL || ops->vidioc_queryctrl == NULL)
		return -EINVAL;

	return ops->vidioc_queryctrl(file, fh, query);
}

static int
isp_video_get_controls(struct file *file, void *fh,
		       struct v4l2_ext_controls *ctrls)
{
	struct isp_video *video = video_drvdata(file);
	const struct v4l2_ioctl_ops *ops = video->ioctl_ops;

	if (ops == NULL || ops->vidioc_g_ext_ctrls == NULL)
		return -EINVAL;

	return ops->vidioc_g_ext_ctrls(file, fh, ctrls);
}

static int
isp_video_set_controls(struct file *file, void *fh,
		       struct v4l2_ext_controls *ctrls)
{
	struct isp_video *video = video_drvdata(file);
	const struct v4l2_ioctl_ops *ops = video->ioctl_ops;

	if (ops == NULL || ops->vidioc_s_ext_ctrls == NULL)
		return -EINVAL;

	return ops->vidioc_s_ext_ctrls(file, fh, ctrls);
}

static int
isp_video_query_menu(struct file *file, void *fh,
		     struct v4l2_querymenu *query)
{
	struct isp_video *video = video_drvdata(file);
	const struct v4l2_ioctl_ops *ops = video->ioctl_ops;

	if (ops == NULL || ops->vidioc_querymenu == NULL)
		return -EINVAL;

	return ops->vidioc_querymenu(file, fh, query);
}

static int
isp_video_cropcap(struct file *file, void *fh, struct v4l2_cropcap *cropcap)
{
	struct isp_video *video = video_drvdata(file);
	struct v4l2_subdev *subdev;
	int ret;

	subdev = isp_video_remote_subdev(video, NULL);
	if (subdev == NULL)
		return -EINVAL;

	mutex_lock(&video->mutex);
	ret = v4l2_subdev_call(subdev, video, cropcap, cropcap);
	mutex_unlock(&video->mutex);

	return ret == -ENOIOCTLCMD ? -EINVAL : ret;
}

static int
isp_video_get_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct isp_video *video = video_drvdata(file);
	struct v4l2_subdev *subdev;
	struct v4l2_mbus_framefmt format;
	u32 pad;
	int ret;

	subdev = isp_video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EINVAL;

	/* Try the get crop operation first and fallback to get format if not
	 * implemented.
	 */
	ret = v4l2_subdev_call(subdev, video, g_crop, crop);
	if (ret != -ENOIOCTLCMD)
		return ret;

	ret = v4l2_subdev_call(subdev, pad, get_fmt, pad, &format,
			       V4L2_SUBDEV_FORMAT_ACTIVE);
	if (ret < 0)
		return ret == -ENOIOCTLCMD ? -EINVAL : ret;

	crop->c.left = 0;
	crop->c.top = 0;
	crop->c.width = format.width;
	crop->c.height = format.height;

	return 0;
}

static int
isp_video_set_crop(struct file *file, void *fh, struct v4l2_crop *crop)
{
	struct isp_video *video = video_drvdata(file);
	struct v4l2_subdev *subdev;
	int ret;

	subdev = isp_video_remote_subdev(video, NULL);
	if (subdev == NULL)
		return -EINVAL;

	mutex_lock(&video->mutex);
	ret = v4l2_subdev_call(subdev, video, s_crop, crop);
	mutex_unlock(&video->mutex);

	return ret == -ENOIOCTLCMD ? -EINVAL : ret;
}

static int
isp_video_get_param(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct isp_video *video = video_drvdata(file);
	const struct v4l2_ioctl_ops *ops = video->ioctl_ops;

	if (ops == NULL || ops->vidioc_g_parm == NULL)
		return -EINVAL;

	return ops->vidioc_g_parm(file, fh, a);
}

static int
isp_video_set_param(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct isp_video *video = video_drvdata(file);
	const struct v4l2_ioctl_ops *ops = video->ioctl_ops;

	if (ops == NULL || ops->vidioc_s_parm == NULL)
		return -EINVAL;

	return ops->vidioc_s_parm(file, fh, a);
}

static int
isp_video_reqbufs(struct file *file, void *fh, struct v4l2_requestbuffers *rb)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);

	return isp_video_queue_reqbufs(&vfh->queue, rb);
}

static int
isp_video_querybuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);

	return isp_video_queue_querybuf(&vfh->queue, b);
}

static int
isp_video_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);

	return isp_video_queue_qbuf(&vfh->queue, b);
}

static int
isp_video_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);

	return isp_video_queue_dqbuf(&vfh->queue, b,
				     file->f_flags & O_NONBLOCK);
}

/*
 * Stream management
 *
 * Every ISP pipeline has a single input and a single output. The input can be
 * either a sensor or a video node. The output is always a video node.
 *
 * As every pipeline has an output video node, the ISP video objects at the
 * pipeline output stores the pipeline state. It tracks the streaming state of
 * both the input and output, as well as the availability of buffers.
 *
 * In sensor-to-memory mode, frames are always available at the pipeline input.
 * Starting the sensor usually requires I2C transfers and must be done in
 * interruptible context. The pipeline is started and stopped synchronously
 * to the stream on/off commands. All modules in the pipeline will get their
 * subdev set stream handler called. The module at the end of the pipeline must
 * delay starting the hardware until buffers are available at its output.
 *
 * In memory-to-memory mode, starting/stopping the stream requires
 * synchronization between the input and output. ISP modules can't be stopped
 * in the middle of a frame, and at least some of the modules seem to become
 * busy as soon as they're started, even if they don't receive a frame start
 * event. For that reason frames need to be processed in single-shot mode. The
 * driver needs to wait until a frame is completely processed and written to
 * memory before restarting the pipeline for the next frame. Pipelined
 * processing might be possible but requires more testing.
 *
 * Stream start must be delayed until buffers are available at both the input
 * and output. The pipeline must be started in the videobuf queue callback with
 * the buffers queue spinlock held. The modules subdev set stream operation must
 * not sleep.
 */
static int
isp_video_streamon(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);
	enum isp_pipeline_state state;
	struct isp_video *far_end;
	unsigned int streaming;
	unsigned long flags;
	int ret;

	if (type != video->type)
		return -EINVAL;

	mutex_lock(&video->stream_lock);

	mutex_lock(&vfh->queue.lock);
	streaming = vfh->queue.streaming;
	mutex_unlock(&vfh->queue.lock);

	if (video->pipe != NULL || streaming) {
		mutex_unlock(&video->stream_lock);
		return -EBUSY;
	}

	video->queue = &vfh->queue;
	INIT_LIST_HEAD(&video->dmaqueue);

	/* Verify that the currently configured format matches the output of
	 * the connected subdev.
	 */
	ret = isp_video_check_format(video, vfh);
	if (ret < 0)
		goto error;

	/* Find the ISP video node connected at the far end of the pipeline. */
	far_end = isp_video_far_end(video);
	atomic_set(&video->sequence, -1);

	/* Update the pipeline state. */
	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		state = ISP_PIPELINE_STREAM_OUTPUT;
		video->pipe = &video->__pipe;
		video->pipe->input = far_end;
		video->pipe->output = video;
	} else {
		state = ISP_PIPELINE_STREAM_INPUT;
		video->pipe = &far_end->__pipe;
		video->pipe->input = video;
		video->pipe->output = far_end;
	}

	spin_lock_irqsave(&video->pipe->lock, flags);
	video->pipe->state |= state;
	spin_unlock_irqrestore(&video->pipe->lock, flags);

	/*
	 * Formula from: resource34xx.c set_opp()
	 * If MPU freq is above 500MHz, make sure the interconnect
	 * is at 100Mhz or above.
	 * throughput in KiB/s for 100 Mhz = 100 * 1000 * 4.
	 *
	 * We want to be fast enough then set OCP clock to be max as
	 * possible, in that case 185Mhz then:
	 * throughput in KiB/s for 185Mhz = 185 * 1000 * 4 = 740000 KiB/s
	 */
	omap_pm_set_min_bus_tput(video->isp->dev, OCP_INITIATOR_AGENT, 740000);

	/* In sensor-to-memory mode, the stream can be started synchronously
	 * to the stream on command. In memory-to-memory mode, it will be
	 * started when buffers are queued on both the input and output.
	 */
	if (video->pipe->input == NULL) {
		ret = isp_pipeline_set_stream(video->isp, video,
					      ISP_PIPELINE_STREAM_CONTINUOUS);
		if (ret < 0)
			goto error;
	}

	ret = isp_video_queue_streamon(&vfh->queue);
	if (ret < 0 && video->pipe->input == NULL) {
		if (video->ops->stream_off)
			video->ops->stream_off(video);
		else
			isp_pipeline_set_stream(video->isp, video,
						ISP_PIPELINE_STREAM_STOPPED);
	}

error:
	if (ret < 0) {
		omap_pm_set_min_bus_tput(video->isp->dev,
					 OCP_INITIATOR_AGENT, 0);
		video->pipe = NULL;
		video->queue = NULL;
	}

	mutex_unlock(&video->stream_lock);
	return ret;
}

static int
isp_video_streamoff(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct isp_video_fh *vfh = to_isp_video_fh(fh);
	struct isp_video *video = video_drvdata(file);
	enum isp_pipeline_state state;
	unsigned int streaming;
	unsigned long flags;

	if (type != video->type)
		return -EINVAL;

	mutex_lock(&video->stream_lock);

	mutex_lock(&vfh->queue.lock);
	streaming = vfh->queue.streaming;
	mutex_unlock(&vfh->queue.lock);

	if (video->pipe == NULL || !streaming)
		goto done;

	/* Update the pipeline state. */
	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE)
		state = ISP_PIPELINE_STREAM_OUTPUT
		      | ISP_PIPELINE_QUEUE_OUTPUT;
	else
		state = ISP_PIPELINE_STREAM_INPUT
		      | ISP_PIPELINE_QUEUE_INPUT;

	spin_lock_irqsave(&video->pipe->lock, flags);
	video->pipe->state &= ~state;
	spin_unlock_irqrestore(&video->pipe->lock, flags);

	/* Stop the stream. */
	if (video->ops->stream_off)
		video->ops->stream_off(video);
	else
		isp_pipeline_set_stream(video->isp, video->pipe->output,
					ISP_PIPELINE_STREAM_STOPPED);

	isp_video_queue_streamoff(&vfh->queue);
	video->pipe = NULL;
	video->queue = NULL;

	omap_pm_set_min_bus_tput(video->isp->dev, OCP_INITIATOR_AGENT, 0);

done:
	mutex_unlock(&video->stream_lock);
	return 0;
}

static int
isp_video_enum_input(struct file *file, void *fh, struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;

	strlcpy(input->name, "camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int
isp_video_g_input(struct file *file, void *fh, unsigned int *input)
{
	*input = 0;

	return 0;
}

static int
isp_video_s_input(struct file *file, void *fh, unsigned int input)
{
	return input == 0 ? 0 : -EINVAL;
}

static long
isp_video_ioctl_default(struct file *file, void *fh, int cmd, void *arg)
{
	struct isp_video *video = video_drvdata(file);
	int ret;

	mutex_lock(&video->mutex);
	ret = isp_handle_private(video->isp, cmd, arg);
	mutex_unlock(&video->mutex);

	return ret;
}

static const struct v4l2_ioctl_ops isp_video_ioctl_ops = {
	.vidioc_querycap		= isp_video_querycap,
	.vidioc_enum_fmt_vid_cap	= isp_video_enum_formats,
	.vidioc_enum_framesizes		= isp_video_enum_framesizes,
	.vidioc_enum_frameintervals	= isp_video_enum_frameintervals,
	.vidioc_g_fmt_vid_cap		= isp_video_get_format,
	.vidioc_s_fmt_vid_cap		= isp_video_set_format,
	.vidioc_try_fmt_vid_cap		= isp_video_try_format,
	.vidioc_g_fmt_vid_out		= isp_video_get_format,
	.vidioc_s_fmt_vid_out		= isp_video_set_format,
	.vidioc_try_fmt_vid_out		= isp_video_try_format,
	.vidioc_queryctrl		= isp_video_query_control,
	.vidioc_g_ext_ctrls		= isp_video_get_controls,
	.vidioc_s_ext_ctrls		= isp_video_set_controls,
	.vidioc_querymenu		= isp_video_query_menu,
	.vidioc_cropcap			= isp_video_cropcap,
	.vidioc_g_crop			= isp_video_get_crop,
	.vidioc_s_crop			= isp_video_set_crop,
	.vidioc_g_parm			= isp_video_get_param,
	.vidioc_s_parm			= isp_video_set_param,
	.vidioc_reqbufs			= isp_video_reqbufs,
	.vidioc_querybuf		= isp_video_querybuf,
	.vidioc_qbuf			= isp_video_qbuf,
	.vidioc_dqbuf			= isp_video_dqbuf,
	.vidioc_streamon		= isp_video_streamon,
	.vidioc_streamoff		= isp_video_streamoff,
	.vidioc_enum_input		= isp_video_enum_input,
	.vidioc_g_input			= isp_video_g_input,
	.vidioc_s_input			= isp_video_s_input,
	.vidioc_default			= isp_video_ioctl_default,
};

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */

static int isp_video_open(struct file *file)
{
	struct isp_video *video = video_drvdata(file);
	struct isp_video_fh *handle;
	int ret = 0;

	handle = kzalloc(sizeof(*handle), GFP_KERNEL);
	if (handle == NULL)
		return -ENOMEM;

	v4l2_fh_init(&handle->vfh, &video->video);
	v4l2_fh_add(&handle->vfh);

	/* If this is the first user, initialise the pipeline. */
	if (atomic_inc_return(&video->users) == 1) {
		if (isp_get(video->isp) == NULL) {
			ret = -EBUSY;
			goto done;
		}

		if (video->ops->init) {
			ret = video->ops->init(video);
			if (ret < 0) {
				isp_put(video->isp);
				goto done;
			}
		}

	}

	isp_video_queue_init(&handle->queue, video->type, &isp_video_queue_ops,
			     video->isp->dev, sizeof(struct isp_buffer));

	memset(&handle->format, 0, sizeof(handle->format));
	handle->format.type = video->type;

	handle->video = video;
	file->private_data = &handle->vfh;

done:
	if (ret < 0) {
		v4l2_fh_del(&handle->vfh);
		atomic_dec(&video->users);
		kfree(handle);
	}

	return ret;
}

static int isp_video_release(struct file *file)
{
	struct isp_video *video = video_drvdata(file);
	struct v4l2_fh *vfh = file->private_data;
	struct isp_video_fh *handle = to_isp_video_fh(vfh);

	/* Disable streaming and free the buffers queue resources. */
	isp_video_streamoff(file, vfh, video->type);

	mutex_lock(&handle->queue.lock);
	isp_video_queue_cleanup(&handle->queue);
	mutex_unlock(&handle->queue.lock);

	/* Release the file handle. */
	v4l2_fh_del(vfh);
	kfree(handle);
	file->private_data = NULL;

	/* If this was the last user, clean up the pipeline. */
	if (atomic_dec_return(&video->users) == 0) {
		if (video->ops->cleanup)
			video->ops->cleanup(video);
		isp_put(video->isp);
	}

	return 0;
}

static unsigned int isp_video_poll(struct file *file, poll_table *wait)
{
	struct isp_video_fh *vfh = to_isp_video_fh(file->private_data);
	struct isp_video_queue *queue = &vfh->queue;

	return isp_video_queue_poll(queue, file, wait);
}

static int isp_video_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct isp_video_fh *vfh = to_isp_video_fh(file->private_data);

	return isp_video_queue_mmap(&vfh->queue, vma);
}

static struct v4l2_file_operations isp_video_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = isp_video_open,
	.release = isp_video_release,
	.poll = isp_video_poll,
	.mmap = isp_video_mmap,
};

/* -----------------------------------------------------------------------------
 * ISP video core
 */

static const struct isp_video_operations isp_video_dummy_ops = {
};

int isp_video_init(struct isp_video *video, const char *name)
{
	const char *direction;
	int ret;

	switch (video->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		direction = "output";
		video->pad.type = MEDIA_PAD_TYPE_INPUT;
		break;
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
		direction = "input";
		video->pad.type = MEDIA_PAD_TYPE_OUTPUT;
		break;

	default:
		return -EINVAL;
	}

	ret = media_entity_init(&video->video.entity, 1, &video->pad, 0);
	if (ret < 0)
		return ret;

	/* FIXME Those three fields are initialized in video_register_device,
	 * but we need them earlier to setup links before all video device nodes
	 * are registered. The removal of legacy nodes in omap34xxcam will
	 * alleviate this constraint.
	 */
	video->video.entity.type = MEDIA_ENTITY_TYPE_NODE;
	video->video.entity.subtype = MEDIA_NODE_TYPE_V4L;
	video->video.entity.name = video->video.name;

	mutex_init(&video->mutex);
	atomic_set(&video->active, 0);
	atomic_set(&video->users, 0);

	spin_lock_init(&video->__pipe.lock);
	mutex_init(&video->stream_lock);

	/* Initialize the video device. */
	if (video->ops == NULL)
		video->ops = &isp_video_dummy_ops;

	video->video.fops = &isp_video_fops;
	snprintf(video->video.name, sizeof(video->video.name),
		 "OMAP3 ISP %s %s", name, direction);
	video->video.vfl_type = VFL_TYPE_GRABBER;
	video->video.release = video_device_release_empty;
	video->video.ioctl_ops = &isp_video_ioctl_ops;

	video_set_drvdata(&video->video, video);

	return 0;
}
EXPORT_SYMBOL_GPL(isp_video_init);

int isp_video_register(struct isp_video *video, struct v4l2_device *vdev)
{
	int ret;

	video->video.v4l2_dev = vdev;

	ret = video_register_device(&video->video, VFL_TYPE_GRABBER, -1);
	if (ret < 0)
		printk(KERN_ERR "%s: could not register video device (%d)\n",
			__func__, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(isp_video_register);

void isp_video_unregister(struct isp_video *video)
{
	if (video_is_registered(&video->video)) {
		media_entity_cleanup(&video->video.entity);
		video_unregister_device(&video->video);
	}
}
EXPORT_SYMBOL_GPL(isp_video_unregister);

