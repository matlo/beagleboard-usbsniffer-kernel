/*
 * omap34xxcam.c
 *
 * Copyright (C) 2006--2009 Nokia Corporation
 * Copyright (C) 2007--2009 Texas Instruments
 *
 * Contact: Sakari Ailus <sakari.ailus@nokia.com>
 *          Tuukka Toivonen <tuukka.o.toivonen@nokia.com>
 *
 * Originally based on the OMAP 2 camera driver.
 *
 * Written by Sakari Ailus <sakari.ailus@nokia.com>
 *            Tuukka Toivonen <tuukka.o.toivonen@nokia.com>
 *            Sergio Aguirre <saaguirre@ti.com>
 *            Mohit Jalori
 *            Sameer Venkatraman
 *            Leonides Martinez
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/videodev2.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/dma-mapping.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-int-device.h>
#include <media/tvp514x-int.h>

#include "omap34xxcam.h"
#include "isp/isp.h"

#define OMAP34XXCAM_VERSION KERNEL_VERSION(0, 0, 0)

/* global variables */
static struct omap34xxcam_device *omap34xxcam;

/*
 *
 * Sensor handling.
 *
 */

/**
 * omap34xxcam_slave_power_set - set slave power state
 * @vdev: per-video device data structure
 * @power: new power state
 */
static int omap34xxcam_slave_power_set(struct omap34xxcam_videodev *vdev,
				       enum v4l2_power power,
				       int mask)
{
	int rval = 0, i = 0;

	BUG_ON(!mutex_is_locked(&vdev->mutex));

#ifdef OMAP34XXCAM_POWEROFF_DELAY
	vdev->power_state_wish = -1;
#endif

	for (i = 0; i <= OMAP34XXCAM_SLAVE_FLASH; i++) {
		if (vdev->slave[i] == v4l2_int_device_dummy())
			continue;

		if (!(mask & (1 << i))
		    || power == vdev->power_state[i])
			continue;

		rval = vidioc_int_s_power(vdev->slave[i], power);

		if (rval && power != V4L2_POWER_OFF) {
			power = V4L2_POWER_OFF;
			goto out;
		}

		vdev->power_state[i] = power;
	}

	return 0;

out:
	for (i--; i >= 0; i--) {
		if (vdev->slave[i] == v4l2_int_device_dummy())
			continue;

		if (!(mask & (1 << i)))
			continue;

		vidioc_int_s_power(vdev->slave[i], power);
		vdev->power_state[i] = power;
	}

	return rval;
}

#ifdef OMAP34XXCAM_POWEROFF_DELAY
static void omap34xxcam_slave_power_work(struct work_struct *work)
{
	struct omap34xxcam_videodev *vdev =
		container_of(work, struct omap34xxcam_videodev, poweroff_work);

	mutex_lock(&vdev->mutex);

	if (vdev->power_state_wish != -1)
		omap34xxcam_slave_power_set(vdev, vdev->power_state_wish,
					    vdev->power_state_mask);

	mutex_unlock(&vdev->mutex);
}

static void omap34xxcam_slave_power_timer(unsigned long ptr)
{
	struct omap34xxcam_videodev *vdev = (void *)ptr;

	schedule_work(&vdev->poweroff_work);
}

/**
 * omap34xxcam_slave_power_suggest - delayed power state change
 *
 * @vdev: per-video device data structure
 * @power: new power state
 */
static void omap34xxcam_slave_power_suggest(struct omap34xxcam_videodev *vdev,
					    enum v4l2_power power,
					    int mask)
{
	BUG_ON(!mutex_is_locked(&vdev->mutex));

	del_timer(&vdev->poweroff_timer);

	vdev->power_state_wish = power;
	vdev->power_state_mask = mask;

	mod_timer(&vdev->poweroff_timer, jiffies + OMAP34XXCAM_POWEROFF_DELAY);
}
#else /* OMAP34XXCAM_POWEROFF_DELAY */
#define omap34xxcam_slave_power_suggest(a, b, c) do {} while (0)
#endif /* OMAP34XXCAM_POWEROFF_DELAY */

/**
 * omap34xxcam_update_vbq - Updates VBQ with completed input buffer
 * @vb: ptr. to standard V4L2 video buffer structure
 *
 * Updates video buffer queue with completed buffer passed as
 * input parameter.  Also updates ISP H3A timestamp and field count
 * statistics.
 */
void omap34xxcam_vbq_complete(struct videobuf_buffer *vb, void *priv)
{
	struct omap34xxcam_fh *fh = priv;

	do_gettimeofday(&vb->ts);
	vb->field_count = atomic_add_return(2, &fh->field_count);

	wake_up(&vb->done);
}

/**
 * omap34xxcam_vbq_setup - Calcs size and num of buffs allowed in queue
 * @vbq: ptr. to standard V4L2 video buffer queue structure
 * @cnt: ptr to location to hold the count of buffers to be in the queue
 * @size: ptr to location to hold the size of a frame
 *
 * Calculates the number of buffers of current image size that can be
 * supported by the available capture memory.
 */
static int omap34xxcam_vbq_setup(struct videobuf_queue *vbq, unsigned int *cnt,
				 unsigned int *size)
{
	struct omap34xxcam_fh *fh = vbq->priv_data;
	struct omap34xxcam_videodev *vdev = fh->vdev;

	if (*cnt <= 0)
		*cnt = VIDEO_MAX_FRAME;	/* supply a default number of buffers */

	if (*cnt > VIDEO_MAX_FRAME)
		*cnt = VIDEO_MAX_FRAME;

	*size = vdev->pix.sizeimage;

	while (*size * *cnt > fh->vdev->vdev_sensor_config.capture_mem)
		(*cnt)--;

	return isp_vbq_setup(vdev->cam->isp, vbq, cnt, size);
}

/**
 * omap34xxcam_vbq_release - Free resources for input VBQ and VB
 * @vbq: ptr. to standard V4L2 video buffer queue structure
 * @vb: ptr to standard V4L2 video buffer structure
 *
 * Unmap and free all memory associated with input VBQ and VB, also
 * unmap the address in ISP MMU.  Reset the VB state.
 */
static void omap34xxcam_vbq_release(struct videobuf_queue *vbq,
				    struct videobuf_buffer *vb)
{
	struct omap34xxcam_fh *fh = vbq->priv_data;
	struct omap34xxcam_videodev *vdev = fh->vdev;
	struct device *isp = vdev->cam->isp;

	if (!vbq->streaming || vb->memory != V4L2_MEMORY_MMAP) {
		isp_vbq_release(isp, vbq, vb);
		videobuf_dma_unmap(vbq, videobuf_to_dma(vb));
		videobuf_dma_free(videobuf_to_dma(vb));
		vb->state = VIDEOBUF_NEEDS_INIT;
	}
}



/*
 * This function is work around for the videobuf_iolock API,
 * for User memory allocated with ioremap (VM_IO flag) the API
 * get_user_pages fails.
 *
 * To fulfill this requirement, we have completely ignored VM layer of
 * Linux, and configuring the ISP MMU with physical address.
 */
static int omap_videobuf_dma_init_user(struct videobuf_buffer *vb,
		unsigned long physp, unsigned long asize)
{
	struct videobuf_dmabuf *dma;
	struct scatterlist *sglist;
	unsigned long data, first, last;
	int i = 0;

	dma = videobuf_to_dma(vb);
	data = vb->baddr;

	first = (data & PAGE_MASK) >> PAGE_SHIFT;
	last  = ((data+asize-1) & PAGE_MASK) >> PAGE_SHIFT;
	dma->offset   = data & ~PAGE_MASK;
	dma->nr_pages = last-first+1;

	dma->direction = DMA_FROM_DEVICE;

	BUG_ON(0 == dma->nr_pages);
	/*
	 * Allocate array of sglen + 1, to add entry of extra page
	 * for input buffer. Driver always uses 0th buffer as input buffer.
	 */
	sglist = vmalloc(dma->nr_pages * sizeof(*sglist));
	if (NULL == sglist)
		return -ENOMEM;

	sg_init_table(sglist, dma->nr_pages);

	sglist[0].offset = 0;
	sglist[0].length = PAGE_SIZE - dma->offset;
	sglist[0].dma_address = (dma_addr_t)physp;
	physp += sglist[0].length;
	/*
	 * Iterate in a loop for the number of pages
	 */
	for (i = 1; i < dma->nr_pages; i++) {
		sglist[i].offset = 0;
		sglist[i].length = PAGE_SIZE;
		sglist[i].dma_address = (dma_addr_t)physp;
		physp += PAGE_SIZE;
	}
	dma->sglist = sglist;
	dma->sglen = dma->nr_pages;

	return 0;

}

/**
 * omap34xxcam_vbq_prepare - V4L2 video ops buf_prepare handler
 * @vbq: ptr. to standard V4L2 video buffer queue structure
 * @vb: ptr to standard V4L2 video buffer structure
 * @field: standard V4L2 field enum
 *
 * Verifies there is sufficient locked memory for the requested
 * buffer, or if there is not, allocates, locks and initializes
 * it.
 */
static int omap34xxcam_vbq_prepare(struct videobuf_queue *vbq,
				   struct videobuf_buffer *vb,
				   enum v4l2_field field)
{
	struct omap34xxcam_fh *fh = vbq->priv_data;
	struct omap34xxcam_videodev *vdev = fh->vdev;
	struct device *isp = vdev->cam->isp;

	int err = 0;

	/*
	 * Accessing pix here is okay since it's constant while
	 * streaming is on (and we only get called then).
	 */
	if (vb->baddr) {
		/* This is a userspace buffer. */
		if (vdev->pix.sizeimage > vb->bsize ||
				vb->baddr != (vb->baddr & ~0x1F))
			/* The buffer isn't big enough. */
			return -EINVAL;
	} else {
		if (vb->state != VIDEOBUF_NEEDS_INIT
		    && vdev->pix.sizeimage > vb->bsize)
			/*
			 * We have a kernel bounce buffer that has
			 * already been allocated.
			 */
			omap34xxcam_vbq_release(vbq, vb);
	}

	vb->size = vdev->pix.bytesperline * vdev->pix.height;
	vb->width = vdev->pix.width;
	vb->height = vdev->pix.height;
	vb->field = field;

	if (vb->state == VIDEOBUF_NEEDS_INIT) {
		struct videobuf_dmabuf *dma;
		struct vm_area_struct *vma;
		dma = videobuf_to_dma(vb);
		vma = find_vma(current->mm, vb->baddr);
		if ((vma) && (vma->vm_flags & VM_IO) && (vma->vm_pgoff)) {
			/* This will catch ioremaped buffers to the kernel.
			 *  It gives two possible scenarios -
			 *  - Driver allocates buffer using either
			 *    dma_alloc_coherent or get_free_pages,
			 *    and maps to user space using
			 *    io_remap_pfn_range/remap_pfn_range
			 *  - Drivers maps memory outside from Linux using
			 *    io_remap
			 */
			unsigned long physp = 0;
			if ((vb->baddr + vb->bsize) > vma->vm_end) {
				dev_err(&vdev->vfd->dev,
						"User Buffer Allocation:" \
						"err=%lu[%u]\n",\
						(vma->vm_end - vb->baddr),
						vb->bsize);
				return -ENOMEM;
			}
			physp = (vma->vm_pgoff << PAGE_SHIFT) +
				(vb->baddr - vma->vm_start);
			err = omap_videobuf_dma_init_user(vb, physp, vb->bsize);
		} else {
			err = videobuf_iolock(vbq, vb, NULL);
		}

		if (!err) {
			/* isp_addr will be stored locally inside isp code */
			err = isp_vbq_prepare(isp, vbq, vb, field);
		}
	}

	if (!err)
		vb->state = VIDEOBUF_PREPARED;
	else
		omap34xxcam_vbq_release(vbq, vb);

	return err;
}

/**
 * omap34xxcam_vbq_queue - V4L2 video ops buf_queue handler
 * @vbq: ptr. to standard V4L2 video buffer queue structure
 * @vb: ptr to standard V4L2 video buffer structure
 *
 * Maps the video buffer to sgdma and through the isp, sets
 * the isp buffer done callback and sets the video buffer state
 * to active.
 */
static void omap34xxcam_vbq_queue(struct videobuf_queue *vbq,
				  struct videobuf_buffer *vb)
{
	struct omap34xxcam_fh *fh = vbq->priv_data;
	struct omap34xxcam_videodev *vdev = fh->vdev;
	struct device *isp = vdev->cam->isp;

	isp_buf_queue(isp, vb, omap34xxcam_vbq_complete, (void *)fh);
}

static struct videobuf_queue_ops omap34xxcam_vbq_ops = {
	.buf_setup = omap34xxcam_vbq_setup,
	.buf_prepare = omap34xxcam_vbq_prepare,
	.buf_queue = omap34xxcam_vbq_queue,
	.buf_release = omap34xxcam_vbq_release,
};

/*
 *
 * IOCTL interface.
 *
 */

/**
 * vidioc_querycap - V4L2 query capabilities IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @cap: ptr to standard V4L2 capability structure
 *
 * Fill in the V4L2 capabliity structure for the camera device
 */
static int vidioc_querycap(struct file *file, void *fh,
			   struct v4l2_capability *cap)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;

	strlcpy(cap->driver, CAM_SHORT_NAME, sizeof(cap->driver));
	strlcpy(cap->card, vdev->vfd->name, sizeof(cap->card));
	cap->version = OMAP34XXCAM_VERSION;
	if (vdev->vdev_sensor != v4l2_int_device_dummy())
		cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;

	return 0;
}

/**
 * vidioc_enum_fmt_vid_cap - V4L2 enumerate format capabilities IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @f: ptr to standard V4L2 format description structure
 *
 * Fills in enumerate format capabilities information for sensor (if SOC
 * sensor attached) or ISP (if raw sensor attached).
 */
static int vidioc_enum_fmt_vid_cap(struct file *file, void *fh,
				   struct v4l2_fmtdesc *f)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	int rval;

	if (vdev->vdev_sensor == v4l2_int_device_dummy())
		return -EINVAL;

	if (vdev->vdev_sensor_mode)
		rval = isp_enum_fmt_cap(f);
	else if (vdev->vdev_sensor_config.sensor_isp)
		rval = vidioc_int_enum_fmt_cap(vdev->vdev_sensor, f);
	else
		rval = isp_enum_fmt_cap(f);

	return rval;
}

/**
 * vidioc_g_fmt_vid_cap - V4L2 get format capabilities IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @f: ptr to standard V4L2 format structure
 *
 * Fills in format capabilities for sensor (if SOC sensor attached) or ISP
 * (if raw sensor attached).
 */
static int vidioc_g_fmt_vid_cap(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;

	if (vdev->vdev_sensor == v4l2_int_device_dummy())
		return -EINVAL;

	mutex_lock(&vdev->mutex);
	f->fmt.pix = vdev->pix;
	mutex_unlock(&vdev->mutex);

	return 0;
}

static int try_pix_parm(struct omap34xxcam_videodev *vdev,
			struct v4l2_pix_format *best_pix_in,
			struct v4l2_pix_format *wanted_pix_out,
			struct v4l2_fract *best_ival)
{
	int fps;
	int fmtd_index;
	int rval;
	struct v4l2_pix_format best_pix_out;
	struct device *isp = vdev->cam->isp;

	if (best_ival->numerator == 0
	    || best_ival->denominator == 0)
		*best_ival = vdev->vdev_sensor_config.ival_default;

	fps = best_ival->denominator / best_ival->numerator;

	memset(best_pix_in, 0, sizeof(*best_pix_in));

	best_ival->denominator = 0;
	best_pix_out.height = INT_MAX >> 1;
	best_pix_out.width = best_pix_out.height;

	for (fmtd_index = 0; ; fmtd_index++) {
		int size_index;
		struct v4l2_fmtdesc fmtd;

		fmtd.index = fmtd_index;
		fmtd.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		rval = vidioc_int_enum_fmt_cap(vdev->vdev_sensor, &fmtd);
		if (rval)
			break;
		dev_dbg(&vdev->vfd->dev, "trying fmt %8.8x (%d)\n",
			fmtd.pixelformat, fmtd_index);
		/*
		 * Get supported resolutions.
		 */
		for (size_index = 0; ; size_index++) {
			struct v4l2_frmsizeenum frms;
			struct v4l2_pix_format pix_tmp_in, pix_tmp_out;
			int ival_index;

			frms.index = size_index;
			frms.pixel_format = fmtd.pixelformat;

			rval = vidioc_int_enum_framesizes(vdev->vdev_sensor,
							  &frms);
			if (rval)
				break;

			pix_tmp_in.pixelformat = frms.pixel_format;
			pix_tmp_in.width = frms.discrete.width;
			pix_tmp_in.height = frms.discrete.height;
			pix_tmp_out = *wanted_pix_out;
			/* Don't do upscaling. */
			if (pix_tmp_out.width > pix_tmp_in.width)
				pix_tmp_out.width = pix_tmp_in.width;
			if (pix_tmp_out.height > pix_tmp_in.height)
				pix_tmp_out.height = pix_tmp_in.height;
			rval = isp_try_fmt_cap(isp, &pix_tmp_in, &pix_tmp_out);
			if (rval)
				return rval;

			dev_dbg(&vdev->vfd->dev, "this w %d\th %d\tfmt %8.8x\t"
				"-> w %d\th %d\t fmt %8.8x"
				"\twanted w %d\th %d\t fmt %8.8x\n",
				pix_tmp_in.width, pix_tmp_in.height,
				pix_tmp_in.pixelformat,
				pix_tmp_out.width, pix_tmp_out.height,
				pix_tmp_out.pixelformat,
				wanted_pix_out->width, wanted_pix_out->height,
				wanted_pix_out->pixelformat);

#define IS_SMALLER_OR_EQUAL(pix1, pix2)				\
			((pix1)->width + (pix1)->height		\
			 < (pix2)->width + (pix2)->height)
#define SIZE_DIFF(pix1, pix2)						\
			(abs((pix1)->width - (pix2)->width)		\
			 + abs((pix1)->height - (pix2)->height))

			/*
			 * Don't use modes that are farther from wanted size
			 * that what we already got.
			 */
			if (SIZE_DIFF(&pix_tmp_out, wanted_pix_out)
			    > SIZE_DIFF(&best_pix_out, wanted_pix_out)) {
				dev_dbg(&vdev->vfd->dev, "size diff bigger: "
					"w %d\th %d\tw %d\th %d\n",
					pix_tmp_out.width, pix_tmp_out.height,
					best_pix_out.width,
					best_pix_out.height);
				continue;
			}

			/*
			 * There's an input mode that can provide output
			 * closer to wanted.
			 */
			if (SIZE_DIFF(&pix_tmp_out, wanted_pix_out)
			    < SIZE_DIFF(&best_pix_out, wanted_pix_out)) {
				/* Force renegotation of fps etc. */
				best_ival->denominator = 0;
				dev_dbg(&vdev->vfd->dev, "renegotiate: "
					"w %d\th %d\tw %d\th %d\n",
					pix_tmp_out.width, pix_tmp_out.height,
					best_pix_out.width,
					best_pix_out.height);
			}

			for (ival_index = 0; ; ival_index++) {
				struct v4l2_frmivalenum frmi;

				frmi.index = ival_index;
				frmi.pixel_format = frms.pixel_format;
				frmi.width = frms.discrete.width;
				frmi.height = frms.discrete.height;
				/* FIXME: try to fix standard... */
				frmi.reserved[0] = 0xdeafbeef;

				rval = vidioc_int_enum_frameintervals(
					vdev->vdev_sensor, &frmi);
				if (rval)
					break;

				dev_dbg(&vdev->vfd->dev, "fps %d\n",
					frmi.discrete.denominator
					/ frmi.discrete.numerator);

				if (best_ival->denominator == 0)
					goto do_it_now;

				if (best_pix_in->width == 0)
					goto do_it_now;

				/*
				 * We aim to use maximum resolution
				 * from the sensor, provided that the
				 * fps is at least as close as on the
				 * current mode.
				 */
#define FPS_ABS_DIFF(fps, ival) abs(fps - (ival).denominator / (ival).numerator)

				/* Select mode with closest fps. */
				if (FPS_ABS_DIFF(fps, frmi.discrete)
				    < FPS_ABS_DIFF(fps, *best_ival)) {
					dev_dbg(&vdev->vfd->dev, "closer fps: "
						"fps %ld\t fps %ld\n",
						FPS_ABS_DIFF(fps,
							      frmi.discrete),
						FPS_ABS_DIFF(fps, *best_ival));
					goto do_it_now;
				}

				/*
				 * Select bigger resolution if it's available
				 * at same fps.
				 */
				if (frmi.width + frmi.height
				    > best_pix_in->width + best_pix_in->height
				    && FPS_ABS_DIFF(fps, frmi.discrete)
				    <= FPS_ABS_DIFF(fps, *best_ival)) {
					dev_dbg(&vdev->vfd->dev, "bigger res, "
						"same fps: "
						"w %d\th %d\tw %d\th %d\n",
						frmi.width, frmi.height,
						best_pix_in->width,
						best_pix_in->height);
					goto do_it_now;
				}

				dev_dbg(&vdev->vfd->dev, "falling through\n");

				continue;

do_it_now:
				*best_ival = frmi.discrete;
				best_pix_out = pix_tmp_out;
				best_pix_in->width = frmi.width;
				best_pix_in->height = frmi.height;
				best_pix_in->pixelformat = frmi.pixel_format;

				dev_dbg(&vdev->vfd->dev,
					"best_pix_in: w %d\th %d\tfmt %8.8x"
					"\tival %d/%d\n",
					best_pix_in->width,
					best_pix_in->height,
					best_pix_in->pixelformat,
					best_ival->numerator,
					best_ival->denominator);
			}
		}
	}

	if (best_ival->denominator == 0)
		return -EINVAL;

	*wanted_pix_out = best_pix_out;

	dev_dbg(&vdev->vfd->dev, "w %d, h %d, fmt %8.8x -> w %d, h %d\n",
		best_pix_in->width, best_pix_in->height,
		best_pix_in->pixelformat,
		best_pix_out.width, best_pix_out.height);

	return 0;
}

static int s_pix_parm(struct omap34xxcam_videodev *vdev,
		      struct v4l2_pix_format *best_pix,
		      struct v4l2_pix_format *pix,
		      struct v4l2_fract *best_ival)
{
	struct device *isp = vdev->cam->isp;
	struct v4l2_streamparm a;
	struct v4l2_format fmt;
	struct v4l2_format old_fmt;
	int rval;

	rval = try_pix_parm(vdev, best_pix, pix, best_ival);
	if (rval)
		return rval;

	rval = isp_s_fmt_cap(isp, best_pix, pix);
	if (rval)
		return rval;

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix = *best_pix;
	vidioc_int_g_fmt_cap(vdev->vdev_sensor, &old_fmt);
	rval = vidioc_int_s_fmt_cap(vdev->vdev_sensor, &fmt);
	if (rval)
		return rval;

	a.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a.parm.capture.timeperframe = *best_ival;
	rval = vidioc_int_s_parm(vdev->vdev_sensor, &a);

	return rval;
}

/**
 * vidioc_s_fmt_vid_cap - V4L2 set format capabilities IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @f: ptr to standard V4L2 format structure
 *
 * Attempts to set input format with the sensor driver (first) and then the
 * ISP.  Returns the return code from vidioc_g_fmt_vid_cap().
 */
static int vidioc_s_fmt_vid_cap(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	struct v4l2_pix_format pix_tmp;
	struct v4l2_fract timeperframe;
	int rval;

	if (vdev->vdev_sensor == v4l2_int_device_dummy())
		return -EINVAL;

	mutex_lock(&vdev->mutex);
	if (vdev->streaming) {
		rval = -EBUSY;
		goto out;
	}

	if (vdev->vdev_sensor_mode) {
		struct v4l2_format input_fmt = *f;
		struct v4l2_pix_format *pix = &f->fmt.pix;
		struct device *isp = vdev->cam->isp;

		rval = isp_try_fmt_cap(isp, pix, pix);
		if (rval)
			goto out;
		/* Always negotiate with the sensor first */
		rval = vidioc_int_s_fmt_cap(vdev->vdev_sensor, &input_fmt);
		if (rval)
			goto out;
		pix->width = input_fmt.fmt.pix.width;
		pix->height = input_fmt.fmt.pix.height;
		pix->pixelformat = input_fmt.fmt.pix.pixelformat;
		pix->field = input_fmt.fmt.pix.field;
		pix->bytesperline = input_fmt.fmt.pix.bytesperline;
		pix->colorspace = input_fmt.fmt.pix.colorspace;
		pix->sizeimage = input_fmt.fmt.pix.sizeimage;
		/* Negotiate with OMAP3 ISP */
		rval = isp_s_fmt_cap(isp, pix, pix);
		if (!rval)
			vdev->pix = f->fmt.pix;
	} else {
		vdev->want_pix = f->fmt.pix;

		timeperframe = vdev->want_timeperframe;

		rval = s_pix_parm(vdev, &pix_tmp, &f->fmt.pix, &timeperframe);
		if (!rval)
			vdev->pix = f->fmt.pix;
	}
out:
	mutex_unlock(&vdev->mutex);

	return rval;
}

/**
 * vidioc_try_fmt_vid_cap - V4L2 try format capabilities IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @f: ptr to standard V4L2 format structure
 *
 * Checks if the given format is supported by the sensor driver and
 * by the ISP.
 */
static int vidioc_try_fmt_vid_cap(struct file *file, void *fh,
				  struct v4l2_format *f)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	struct v4l2_pix_format pix_tmp;
	struct v4l2_fract timeperframe;
	int rval;

	if (vdev->vdev_sensor == v4l2_int_device_dummy())
		return -EINVAL;

	mutex_lock(&vdev->mutex);

	timeperframe = vdev->want_timeperframe;

	rval = try_pix_parm(vdev, &pix_tmp, &f->fmt.pix, &timeperframe);

	mutex_unlock(&vdev->mutex);

	return rval;
}

/**
 * vidioc_reqbufs - V4L2 request buffers IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @b: ptr to standard V4L2 request buffers structure
 *
 * Attempts to get a buffer from the buffer queue associated with the
 * fh through the video buffer library API.
 */
static int vidioc_reqbufs(struct file *file, void *fh,
			  struct v4l2_requestbuffers *b)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	int rval;

	if (vdev->vdev_sensor == v4l2_int_device_dummy())
		return -EINVAL;

	mutex_lock(&vdev->mutex);
	if (vdev->streaming) {
		mutex_unlock(&vdev->mutex);
		return -EBUSY;
	}

	rval = videobuf_reqbufs(&ofh->vbq, b);

	mutex_unlock(&vdev->mutex);

	/*
	 * Either videobuf_reqbufs failed or the buffers are not
	 * memory-mapped (which would need special attention).
	 */
	if (rval < 0 || b->memory != V4L2_MEMORY_MMAP)
		goto out;

out:
	return rval;
}

/**
 * vidioc_querybuf - V4L2 query buffer IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @b: ptr to standard V4L2 buffer structure
 *
 * Attempts to fill in the v4l2_buffer structure for the buffer queue
 * associated with the fh through the video buffer library API.
 */
static int vidioc_querybuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct omap34xxcam_fh *ofh = fh;

	return videobuf_querybuf(&ofh->vbq, b);
}

/**
 * vidioc_qbuf - V4L2 queue buffer IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @b: ptr to standard V4L2 buffer structure
 *
 * Attempts to queue the v4l2_buffer on the buffer queue
 * associated with the fh through the video buffer library API.
 */
static int vidioc_qbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct omap34xxcam_fh *ofh = fh;

	return videobuf_qbuf(&ofh->vbq, b);
}

/**
 * vidioc_dqbuf - V4L2 dequeue buffer IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @b: ptr to standard V4L2 buffer structure
 *
 * Attempts to dequeue the v4l2_buffer from the buffer queue
 * associated with the fh through the video buffer library API.  If the
 * buffer is a user space buffer, then this function will also requeue it,
 * as user does not expect to do this.
 */
static int vidioc_dqbuf(struct file *file, void *fh, struct v4l2_buffer *b)
{
	struct omap34xxcam_fh *ofh = fh;
	int rval;

videobuf_dqbuf_again:
	rval = videobuf_dqbuf(&ofh->vbq, b, file->f_flags & O_NONBLOCK);

	/*
	 * This is a hack. We don't want to show -EIO to the user
	 * space. Requeue the buffer and try again if we're not doing
	 * this in non-blocking mode.
	 */
	if (rval == -EIO) {
		videobuf_qbuf(&ofh->vbq, b);
		if (!(file->f_flags & O_NONBLOCK))
			goto videobuf_dqbuf_again;
		/*
		 * We don't have a videobuf_buffer now --- maybe next
		 * time...
		 */
		rval = -EAGAIN;
	}

	return rval;
}

/**
 * vidioc_streamon - V4L2 streamon IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @i: V4L2 buffer type
 *
 * Attempts to start streaming by enabling the sensor interface and turning
 * on video buffer streaming through the video buffer library API.  Upon
 * success the function returns 0, otherwise an error code is returned.
 */
static int vidioc_streamon(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	struct device *isp = vdev->cam->isp;
	int rval;

	if (vdev->vdev_sensor == v4l2_int_device_dummy())
		return -EINVAL;

	mutex_lock(&vdev->mutex);
	if (vdev->streaming) {
		rval = -EBUSY;
		goto out;
	}

	rval = omap34xxcam_slave_power_set(vdev, V4L2_POWER_ON,
					   OMAP34XXCAM_SLAVE_POWER_SENSOR_LENS);
	if (rval) {
		dev_dbg(&vdev->vfd->dev,
			"omap34xxcam_slave_power_set failed\n");
		goto out;
	}

	isp_start(isp);

	rval = videobuf_streamon(&ofh->vbq);
	if (rval) {
		isp_stop(isp);
		omap34xxcam_slave_power_set(
			vdev, V4L2_POWER_OFF,
			OMAP34XXCAM_SLAVE_POWER_SENSOR_LENS);
	} else
		vdev->streaming = file;

out:
	mutex_unlock(&vdev->mutex);

	return rval;
}

/**
 * vidioc_streamoff - V4L2 streamoff IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @i: V4L2 buffer type
 *
 * Attempts to stop streaming by flushing all scheduled work, waiting on
 * any queued buffers to complete and then stopping the ISP and turning
 * off video buffer streaming through the video buffer library API.  Upon
 * success the function returns 0, otherwise an error code is returned.
 */
static int vidioc_streamoff(struct file *file, void *fh, enum v4l2_buf_type i)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	struct device *isp = vdev->cam->isp;
	struct videobuf_queue *q = &ofh->vbq;
	int rval;

	mutex_lock(&vdev->mutex);

	if (vdev->streaming == file)
		isp_stop(isp);

	rval = videobuf_streamoff(q);
	if (!rval) {
		vdev->streaming = NULL;

		omap34xxcam_slave_power_set(vdev, V4L2_POWER_STANDBY,
					    OMAP34XXCAM_SLAVE_POWER_SENSOR);
		omap34xxcam_slave_power_suggest(vdev, V4L2_POWER_STANDBY,
						OMAP34XXCAM_SLAVE_POWER_LENS);
	}

	mutex_unlock(&vdev->mutex);

	return rval;
}

/**
 * vidioc_enum_input - V4L2 enumerate input IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @inp: V4L2 input type information structure
 *
 * Fills in v4l2_input structure.  Returns 0.
 */
static int vidioc_enum_input(struct file *file, void *fh,
			     struct v4l2_input *inp)
{
	struct omap34xxcam_videodev *vdev = ((struct omap34xxcam_fh *)fh)->vdev;

	if (vdev->vdev_sensor_mode) {
		if (inp->index == 0) {
			strlcpy(inp->name, "COMPOSITE", sizeof(inp->name));
			inp->type = V4L2_INPUT_TYPE_CAMERA;
		} else if (inp->index == 1) {
			strlcpy(inp->name, "S-VIDEO", sizeof(inp->name));
			inp->type = V4L2_INPUT_TYPE_CAMERA;
		} else
			return -EINVAL;
	} else {
		if (inp->index > 0)
			return -EINVAL;
		strlcpy(inp->name, "camera", sizeof(inp->name));
		inp->type = V4L2_INPUT_TYPE_CAMERA;
	}

	return 0;
}

/**
 * vidioc_g_input - V4L2 get input IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @i: address to hold index of input supported
 *
 * Sets index to 0.
 */
static int vidioc_g_input(struct file *file, void *fh, unsigned int *i)
{
	struct omap34xxcam_videodev *vdev = ((struct omap34xxcam_fh *)fh)->vdev;
	int rval = 0;

	mutex_lock(&vdev->mutex);
	if (vdev->vdev_sensor_mode) {
		if (vdev->slave_config[OMAP34XXCAM_SLAVE_SENSOR].cur_input
				== INPUT_CVBS_VI4A)
			*i = 0;
		else if (vdev->slave_config[OMAP34XXCAM_SLAVE_SENSOR].cur_input
				== INPUT_SVIDEO_VI2C_VI1C)
			*i = 1;
	} else {
		*i = 0;
	}
	mutex_unlock(&vdev->mutex);

	return rval;
}

/**
 * vidioc_s_input - V4L2 set input IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @i: index of input selected
 *
 * 0 is only index supported.
 */
static int vidioc_s_input(struct file *file, void *fh, unsigned int i)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	int rval = 0;
	struct v4l2_routing route;

	mutex_lock(&vdev->mutex);
	if (vdev->vdev_sensor_mode) {
		if (i == 0)
			route.input = INPUT_CVBS_VI4A;
		else
			route.input = INPUT_SVIDEO_VI2C_VI1C;

		route.output = 0;
		rval = vidioc_int_s_video_routing(vdev->vdev_sensor, &route);
		if (!rval)
			vdev->slave_config[OMAP34XXCAM_SLAVE_SENSOR].cur_input
				= route.input;
	} else {
		if (i > 0)
			rval = -EINVAL;
	}
	mutex_unlock(&vdev->mutex);

	return rval;
}

/**
 * vidioc_queryctrl - V4L2 query control IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 query control ioctl structure
 *
 * If the requested control is supported, returns the control information
 * in the v4l2_queryctrl structure.  Otherwise, returns -EINVAL if the
 * control is not supported.  If the sensor being used is a "smart sensor",
 * this request is passed to the sensor driver, otherwise the ISP is
 * queried and if it does not support the requested control, the request
 * is forwarded to the "raw" sensor driver to see if it supports it.
 */
static int vidioc_queryctrl(struct file *file, void *fh,
			    struct v4l2_queryctrl *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	struct v4l2_queryctrl a_tmp;
	int best_slave = -1;
	u32 best_ctrl = (u32)-1;
	int i;

	if (vdev->vdev_sensor_config.sensor_isp)
		return vidioc_int_queryctrl(vdev->vdev_sensor, a);

	/* No next flags: try slaves directly. */
	if (!(a->id & V4L2_CTRL_FLAG_NEXT_CTRL)) {
		for (i = 0; i <= OMAP34XXCAM_SLAVE_FLASH; i++) {
			if (!vidioc_int_queryctrl(vdev->slave[i], a))
				return 0;
		}
		return isp_queryctrl(a);
	}

	/* Find slave with smallest next control id. */
	for (i = 0; i <= OMAP34XXCAM_SLAVE_FLASH; i++) {
		a_tmp = *a;

		if (vidioc_int_queryctrl(vdev->slave[i], &a_tmp))
			continue;

		if (a_tmp.id < best_ctrl) {
			best_slave = i;
			best_ctrl = a_tmp.id;
		}
	}

	a_tmp = *a;
	if (!isp_queryctrl(&a_tmp)) {
		if (a_tmp.id < best_ctrl) {
			*a = a_tmp;

			return 0;
		}
	}

	if (best_slave == -1)
		return -EINVAL;

	a->id = best_ctrl;
	return vidioc_int_queryctrl(vdev->slave[best_slave], a);
}

/**
 * vidioc_querymenu - V4L2 query menu IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 query menu ioctl structure
 *
 * If the requested control is supported, returns the menu information
 * in the v4l2_querymenu structure.  Otherwise, returns -EINVAL if the
 * control is not supported or is not a menu.  If the sensor being used
 * is a "smart sensor", this request is passed to the sensor driver,
 * otherwise the ISP is queried and if it does not support the requested
 * menu control, the request is forwarded to the "raw" sensor driver to
 * see if it supports it.
 */
static int vidioc_querymenu(struct file *file, void *fh,
			    struct v4l2_querymenu *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	int i;

	if (vdev->vdev_sensor_config.sensor_isp)
		return vidioc_int_querymenu(vdev->vdev_sensor, a);

	/* Try slaves directly. */
	for (i = 0; i <= OMAP34XXCAM_SLAVE_FLASH; i++) {
		if (!vidioc_int_querymenu(vdev->slave[i], a))
			return 0;
	}
	return isp_querymenu(a);
}

static int vidioc_g_ext_ctrls(struct file *file, void *fh,
			      struct v4l2_ext_controls *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	struct device *isp = vdev->cam->isp;
	int i, ctrl_idx, rval = 0;

	mutex_lock(&vdev->mutex);

	for (ctrl_idx = 0; ctrl_idx < a->count; ctrl_idx++) {
		struct v4l2_control ctrl;

		ctrl.id = a->controls[ctrl_idx].id;

		if (vdev->vdev_sensor_config.sensor_isp) {
			rval = vidioc_int_g_ctrl(vdev->vdev_sensor, &ctrl);
		} else {
			for (i = 0; i <= OMAP34XXCAM_SLAVE_FLASH; i++) {
				rval = vidioc_int_g_ctrl(vdev->slave[i], &ctrl);
				if (!rval)
					break;
			}
		}

		if (rval)
			rval = isp_g_ctrl(isp, &ctrl);

		if (rval) {
			a->error_idx = ctrl_idx;
			break;
		}

		a->controls[ctrl_idx].value = ctrl.value;
	}

	mutex_unlock(&vdev->mutex);

	return rval;
}

static int vidioc_s_ext_ctrls(struct file *file, void *fh,
			      struct v4l2_ext_controls *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	struct device *isp = vdev->cam->isp;
	int i, ctrl_idx, rval = 0;

	mutex_lock(&vdev->mutex);

	for (ctrl_idx = 0; ctrl_idx < a->count; ctrl_idx++) {
		struct v4l2_control ctrl;

		ctrl.id = a->controls[ctrl_idx].id;
		ctrl.value = a->controls[ctrl_idx].value;

		if (vdev->vdev_sensor_config.sensor_isp) {
			rval = vidioc_int_s_ctrl(vdev->vdev_sensor, &ctrl);
		} else {
			for (i = 0; i <= OMAP34XXCAM_SLAVE_FLASH; i++) {
				rval = vidioc_int_s_ctrl(vdev->slave[i], &ctrl);
				if (!rval)
					break;
			}
		}

		if (rval)
			rval = isp_s_ctrl(isp, &ctrl);

		if (rval) {
			a->error_idx = ctrl_idx;
			break;
		}

		a->controls[ctrl_idx].value = ctrl.value;
	}

	mutex_unlock(&vdev->mutex);

	return rval;
}

/**
 * vidioc_g_parm - V4L2 get parameters IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 stream parameters structure
 *
 * If request is for video capture buffer type, handles request by
 * forwarding to sensor driver.
 */
static int vidioc_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	int rval;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	mutex_lock(&vdev->mutex);
	rval = vidioc_int_g_parm(vdev->vdev_sensor, a);
	mutex_unlock(&vdev->mutex);

	return rval;
}

/**
 * vidioc_s_parm - V4L2 set parameters IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 stream parameters structure
 *
 * If request is for video capture buffer type, handles request by
 * first getting current stream parameters from sensor, then forwarding
 * request to set new parameters to sensor driver.  It then attempts to
 * enable the sensor interface with the new parameters.  If this fails, it
 * reverts back to the previous parameters.
 */
static int vidioc_s_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	struct v4l2_pix_format pix_tmp_sensor, pix_tmp;
	int rval;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	if (vdev->vdev_sensor == v4l2_int_device_dummy())
		return -EINVAL;

	mutex_lock(&vdev->mutex);
	if (vdev->streaming) {
		rval = -EBUSY;
		goto out;
	}

	vdev->want_timeperframe = a->parm.capture.timeperframe;

	pix_tmp = vdev->want_pix;

	rval = s_pix_parm(vdev, &pix_tmp_sensor, &pix_tmp,
			  &a->parm.capture.timeperframe);

out:
	mutex_unlock(&vdev->mutex);

	return rval;
}

/**
 * vidioc_cropcap - V4L2 crop capture IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 crop capture structure
 *
 * If using a "smart" sensor, just forwards request to the sensor driver,
 * otherwise fills in the v4l2_cropcap values locally.
 */
static int vidioc_cropcap(struct file *file, void *fh, struct v4l2_cropcap *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	struct v4l2_cropcap *cropcap = a;
	int rval;

	if (vdev->vdev_sensor == v4l2_int_device_dummy())
		return -EINVAL;

	mutex_lock(&vdev->mutex);

	rval = vidioc_int_cropcap(vdev->vdev_sensor, a);

	if (rval && !vdev->vdev_sensor_config.sensor_isp) {
		struct v4l2_format f;

		/* cropcap failed, try to do this via g_fmt_cap */
		rval = vidioc_int_g_fmt_cap(vdev->vdev_sensor, &f);
		if (!rval) {
			cropcap->bounds.top = 0;
			cropcap->bounds.left = 0;
			cropcap->bounds.width = f.fmt.pix.width;
			cropcap->bounds.height = f.fmt.pix.height;
			cropcap->defrect = cropcap->bounds;
			cropcap->pixelaspect.numerator = 1;
			cropcap->pixelaspect.denominator = 1;
		}
	}

	mutex_unlock(&vdev->mutex);

	return rval;
}

/**
 * vidioc_g_crop - V4L2 get capture crop IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 crop structure
 *
 * If using a "smart" sensor, just forwards request to the sensor driver,
 * otherwise calls the isp functions to fill in current crop values.
 */
static int vidioc_g_crop(struct file *file, void *fh, struct v4l2_crop *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	struct device *isp = vdev->cam->isp;
	int rval = 0;

	if (vdev->vdev_sensor == v4l2_int_device_dummy())
		return -EINVAL;

	mutex_lock(&vdev->mutex);

	if (vdev->vdev_sensor_config.sensor_isp)
		rval = vidioc_int_g_crop(vdev->vdev_sensor, a);
	else
		rval = isp_g_crop(isp, a);

	mutex_unlock(&vdev->mutex);

	return rval;
}

/**
 * vidioc_s_crop - V4L2 set capture crop IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @a: standard V4L2 crop structure
 *
 * If using a "smart" sensor, just forwards request to the sensor driver,
 * otherwise calls the isp functions to set the current crop values.
 */
static int vidioc_s_crop(struct file *file, void *fh, struct v4l2_crop *a)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	struct device *isp = vdev->cam->isp;
	int rval = 0;

	if (vdev->vdev_sensor == v4l2_int_device_dummy())
		return -EINVAL;

	mutex_lock(&vdev->mutex);

	if (vdev->vdev_sensor_config.sensor_isp)
		rval = vidioc_int_s_crop(vdev->vdev_sensor, a);
	else
		rval = isp_s_crop(isp, a);

	mutex_unlock(&vdev->mutex);

	return rval;
}

static int vidioc_enum_framesizes(struct file *file, void *fh,
				  struct v4l2_frmsizeenum *frms)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	u32 pixel_format;
	int rval;

	mutex_lock(&vdev->mutex);

	if (vdev->vdev_sensor_config.sensor_isp) {
		rval = vidioc_int_enum_framesizes(vdev->vdev_sensor, frms);
	} else {
		pixel_format = frms->pixel_format;
		frms->pixel_format = -1;	/* ISP does format conversion */
		rval = vidioc_int_enum_framesizes(vdev->vdev_sensor, frms);
		frms->pixel_format = pixel_format;
	}

	mutex_unlock(&vdev->mutex);
	return rval;
}

static int vidioc_enum_frameintervals(struct file *file, void *fh,
				      struct v4l2_frmivalenum *frmi)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	u32 pixel_format;
	int rval;

	mutex_lock(&vdev->mutex);

	if (vdev->vdev_sensor_config.sensor_isp) {
		rval = vidioc_int_enum_frameintervals(vdev->vdev_sensor, frmi);
	} else {
		pixel_format = frmi->pixel_format;
		frmi->pixel_format = -1;	/* ISP does format conversion */
		rval = vidioc_int_enum_frameintervals(vdev->vdev_sensor, frmi);
		frmi->pixel_format = pixel_format;
	}

	mutex_unlock(&vdev->mutex);
	return rval;
}

/**
 * vidioc_querystd - V4L2 query current standard IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @std: standard V4L2 v4l2_std_id enum
 *
 * If using a "smart" sensor, just forwards request to the sensor driver,
 * otherwise returns error
 */
static int vidioc_querystd(struct file *file, void *fh, v4l2_std_id *std)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	int rval = 0;

	mutex_lock(&vdev->mutex);
	if (vdev->vdev_sensor_mode) {
		rval = vidioc_int_querystd(vdev->vdev_sensor, std);
		if (rval == 0)
			vdev->vfd->current_norm = *std;
	} else
		rval = -EINVAL;
	mutex_unlock(&vdev->mutex);

	return rval;
}

/**
 * vidioc_s_std - V4L2 set standard IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @std: standard V4L2 v4l2_std_id enum
 *
 * If using a "smart" sensor, just forwards request to the sensor driver,
 * otherwise returns error
 */
static int vidioc_s_std(struct file *file, void *fh, v4l2_std_id *std)
{
	struct omap34xxcam_fh *ofh = fh;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	int rval = 0;

	mutex_lock(&vdev->mutex);
	if (vdev->vdev_sensor_mode) {
		rval = vidioc_int_s_std(vdev->vdev_sensor, std);
		if (rval == 0)
			vdev->vfd->current_norm = *std;
	} else
		rval = -EINVAL;
	mutex_unlock(&vdev->mutex);

	return rval;
}

/**
 * vidioc_default - private IOCTL handler
 * @file: ptr. to system file structure
 * @fh: ptr to hold address of omap34xxcam_fh struct (per-filehandle data)
 * @cmd: ioctl cmd value
 * @arg: ioctl arg value
 *
 * If the sensor being used is a "smart sensor", this request is returned to
 * caller with -EINVAL err code.  Otherwise if the control id is the private
 * VIDIOC_PRIVATE_ISP_AEWB_REQ to update the analog gain or exposure,
 * then this request is forwared directly to the sensor to incorporate the
 * feedback. The request is then passed on to the ISP private IOCTL handler,
 * isp_handle_private()
 */
static long vidioc_default(struct file *file, void *fh, int cmd, void *arg)
{
	struct omap34xxcam_fh *ofh = file->private_data;
	struct omap34xxcam_videodev *vdev = ofh->vdev;
	struct device *isp = vdev->cam->isp;
	int rval;

	if (cmd == VIDIOC_PRIVATE_OMAP34XXCAM_SENSOR_INFO) {
		u32 pixclk;
		struct v4l2_pix_format active_size, full_size;
		struct omap34xxcam_sensor_info *ret_sensor_info;

		ret_sensor_info = (struct omap34xxcam_sensor_info *)arg;
		mutex_lock(&vdev->mutex);
		rval = vidioc_int_priv_g_pixclk(vdev->vdev_sensor, &pixclk);
		mutex_unlock(&vdev->mutex);
		if (rval)
			goto out;
		mutex_lock(&vdev->mutex);
		rval = vidioc_int_priv_g_activesize(vdev->vdev_sensor,
					    &active_size);
		mutex_unlock(&vdev->mutex);
		if (rval)
			goto out;
		mutex_lock(&vdev->mutex);
		rval = vidioc_int_priv_g_fullsize(vdev->vdev_sensor,
						  &full_size);
		mutex_unlock(&vdev->mutex);
		if (rval)
			goto out;
		ret_sensor_info->current_xclk = pixclk;
		memcpy(&ret_sensor_info->active_size, &active_size,
			sizeof(struct v4l2_pix_format));
		memcpy(&ret_sensor_info->full_size, &full_size,
			sizeof(struct v4l2_pix_format));
		rval = 0;
		goto out;
	}

	if (vdev->vdev_sensor_config.sensor_isp) {
		rval = -EINVAL;
	} else {
		switch (cmd) {
		case VIDIOC_PRIVATE_ISP_AEWB_REQ:
		{
			/* Need to update sensor first */
			struct isph3a_aewb_data *data;
			struct v4l2_control vc;

			data = (struct isph3a_aewb_data *) arg;
			if (data->update & SET_EXPOSURE) {
				dev_dbg(&vdev->vfd->dev, "using "
					"VIDIOC_PRIVATE_ISP_AEWB_REQ to set "
					"exposure is deprecated!\n");
				vc.id = V4L2_CID_EXPOSURE;
				vc.value = data->shutter;
				mutex_lock(&vdev->mutex);
				rval = vidioc_int_s_ctrl(vdev->vdev_sensor,
							 &vc);
				mutex_unlock(&vdev->mutex);
				if (rval)
					goto out;
			}
			if (data->update & SET_ANALOG_GAIN) {
				dev_dbg(&vdev->vfd->dev, "using "
					"VIDIOC_PRIVATE_ISP_AEWB_REQ to set "
					"gain is deprecated!\n");
				vc.id = V4L2_CID_GAIN;
				vc.value = data->gain;
				mutex_lock(&vdev->mutex);
				rval = vidioc_int_s_ctrl(vdev->vdev_sensor,
							 &vc);
				mutex_unlock(&vdev->mutex);
				if (rval)
					goto out;
			}
		}
		break;
		case VIDIOC_PRIVATE_ISP_AF_REQ: {
			/* Need to update lens first */
			struct isp_af_data *data;
			struct v4l2_control vc;

			if (!vdev->vdev_lens) {
				rval = -EINVAL;
				goto out;
			}
			data = (struct isp_af_data *) arg;
			if (data->update & LENS_DESIRED_POSITION) {
				dev_dbg(&vdev->vfd->dev, "using "
					"VIDIOC_PRIVATE_ISP_AF_REQ to set "
					"lens position is deprecated!\n");
				vc.id = V4L2_CID_FOCUS_ABSOLUTE;
				vc.value = data->desired_lens_direction;
				mutex_lock(&vdev->mutex);
				rval = vidioc_int_s_ctrl(vdev->vdev_lens, &vc);
				mutex_unlock(&vdev->mutex);
				if (rval)
					goto out;
			}
		}
			break;
		}

		mutex_lock(&vdev->mutex);
		rval = isp_handle_private(isp, cmd, arg);
		mutex_unlock(&vdev->mutex);
	}
out:
	return rval;
}

/*
 *
 * File operations.
 *
 */

/**
 * omap34xxcam_poll - file operations poll handler
 * @file: ptr. to system file structure
 * @wait: system poll table structure
 *
 */
static unsigned int omap34xxcam_poll(struct file *file,
				     struct poll_table_struct *wait)
{
	struct omap34xxcam_fh *fh = file->private_data;
	struct omap34xxcam_videodev *vdev = fh->vdev;
	struct videobuf_buffer *vb;

	mutex_lock(&vdev->mutex);
	if (vdev->streaming != file) {
		mutex_unlock(&vdev->mutex);
		return POLLERR;
	}
	mutex_unlock(&vdev->mutex);

	mutex_lock(&fh->vbq.vb_lock);
	if (list_empty(&fh->vbq.stream)) {
		mutex_unlock(&fh->vbq.vb_lock);
		return POLLERR;
	}
	vb = list_entry(fh->vbq.stream.next, struct videobuf_buffer, stream);
	mutex_unlock(&fh->vbq.vb_lock);

	poll_wait(file, &vb->done, wait);

	if (vb->state == VIDEOBUF_DONE || vb->state == VIDEOBUF_ERROR)
		return POLLIN | POLLRDNORM;

	return 0;
}

/**
 * omap34xxcam_mmap - file operations mmap handler
 * @file: ptr. to system file structure
 * @vma: system virt. mem. area structure
 *
 * Maps a virtual memory area via the video buffer API
 */
static int omap34xxcam_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct omap34xxcam_fh *fh = file->private_data;
	return videobuf_mmap_mapper(&fh->vbq, vma);
}

/**
 * omap34xxcam_open - file operations open handler
 * @inode: ptr. to system inode structure
 * @file: ptr. to system file structure
 *
 * Allocates and initializes the per-filehandle data (omap34xxcam_fh),
 * enables the sensor, opens/initializes the ISP interface and the
 * video buffer queue.  Note that this function will allow multiple
 * file handles to be open simultaneously, however only the first
 * handle opened will initialize the ISP.  It is the application
 * responsibility to only use one handle for streaming and the others
 * for control only.
 * This function returns 0 upon success and -ENODEV upon error.
 */
static int omap34xxcam_open(struct file *file)
{
	int rval = 0;
	struct omap34xxcam_videodev *vdev = NULL;
	struct omap34xxcam_device *cam = omap34xxcam;
	struct device *isp;
	struct omap34xxcam_fh *fh;
	struct v4l2_format sensor_format;
	int first_user = 0;
	int i;

	for (i = 0; i < OMAP34XXCAM_VIDEODEVS; i++) {
		if (cam->vdevs[i].vfd
		    && cam->vdevs[i].vfd->minor ==
		    iminor(file->f_dentry->d_inode)) {
			vdev = &cam->vdevs[i];
			break;
		}
	}

	if (!vdev || !vdev->vfd)
		return -ENODEV;

	fh = kzalloc(sizeof(*fh), GFP_KERNEL);
	if (fh == NULL)
		return -ENOMEM;

	fh->vdev = vdev;

	mutex_lock(&vdev->mutex);
	for (i = 0; i <= OMAP34XXCAM_SLAVE_FLASH; i++) {
		if (vdev->slave[i] != v4l2_int_device_dummy()
		    && !try_module_get(vdev->slave[i]->module)) {
			mutex_unlock(&vdev->mutex);
			dev_err(&vdev->vfd->dev, "can't try_module_get %s\n",
				vdev->slave[i]->name);
			rval = -ENODEV;
			goto out_try_module_get;
		}
	}

	if (atomic_inc_return(&vdev->users) == 1) {
		first_user = 1;
		isp = isp_get();
		if (!isp) {
			rval = -EBUSY;
			dev_err(&vdev->vfd->dev, "can't get isp\n");
			goto out_isp_get;
		}
		cam->isp = isp;
		if (omap34xxcam_slave_power_set(vdev, V4L2_POWER_ON,
						OMAP34XXCAM_SLAVE_POWER_ALL)) {
			dev_err(&vdev->vfd->dev, "can't power up slaves\n");
			rval = -EBUSY;
			goto out_slave_power_set_standby;
		}
		omap34xxcam_slave_power_set(
			vdev, V4L2_POWER_STANDBY,
			OMAP34XXCAM_SLAVE_POWER_SENSOR);
		omap34xxcam_slave_power_suggest(
			vdev, V4L2_POWER_STANDBY,
			OMAP34XXCAM_SLAVE_POWER_LENS);
	}

	if (vdev->vdev_sensor == v4l2_int_device_dummy() || !first_user)
		goto out_no_pix;

	if (vdev->vdev_sensor_config.sensor_isp) {
		if ((vdev->slave_config[OMAP34XXCAM_SLAVE_SENSOR].cur_input
					!= INPUT_CVBS_VI4A) &&
				(vdev->slave_config[OMAP34XXCAM_SLAVE_SENSOR].
				 cur_input != INPUT_SVIDEO_VI2C_VI1C)) {
			struct v4l2_routing route;
			int rval;
			route.input = INPUT_CVBS_VI4A;
			route.output = 0;
			rval = vidioc_int_s_video_routing(vdev->vdev_sensor,
					&route);
			if (rval) {
				route.input = INPUT_SVIDEO_VI2C_VI1C;
				rval = vidioc_int_s_video_routing(
						vdev->vdev_sensor, &route);
			}
			if (!rval)
				vdev->slave_config[OMAP34XXCAM_SLAVE_SENSOR]
					.cur_input = route.input;
		}
		sensor_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	}

	/* Get the format the sensor is using. */
	rval = vidioc_int_g_fmt_cap(vdev->vdev_sensor, &sensor_format);
	if (rval) {
		dev_err(&vdev->vfd->dev,
			"can't get current pix from sensor!\n");
		goto out_vidioc_int_g_fmt_cap;
	}

	if (!vdev->pix.width)
		vdev->pix = sensor_format.fmt.pix;

	if (!vdev->vdev_sensor_config.sensor_isp) {
		struct v4l2_pix_format pix;
		struct v4l2_fract timeperframe =
			vdev->want_timeperframe;

		rval = s_pix_parm(vdev, &pix, &vdev->pix, &timeperframe);
		if (rval) {
			dev_err(&vdev->vfd->dev,
				"isp doesn't like the sensor!\n");
			goto out_isp_s_fmt_cap;
		}
	}

out_no_pix:
	mutex_unlock(&vdev->mutex);

	file->private_data = fh;

	spin_lock_init(&fh->vbq_lock);

	videobuf_queue_sg_init(&fh->vbq, &omap34xxcam_vbq_ops, NULL,
				&fh->vbq_lock, V4L2_BUF_TYPE_VIDEO_CAPTURE,
				V4L2_FIELD_NONE,
				sizeof(struct videobuf_buffer), fh);

	return 0;

out_isp_s_fmt_cap:
out_vidioc_int_g_fmt_cap:
	omap34xxcam_slave_power_set(vdev, V4L2_POWER_OFF,
				    OMAP34XXCAM_SLAVE_POWER_ALL);
out_slave_power_set_standby:
	isp_put();

out_isp_get:
	atomic_dec(&vdev->users);
	mutex_unlock(&vdev->mutex);

out_try_module_get:
	for (i--; i >= 0; i--)
		if (vdev->slave[i] != v4l2_int_device_dummy())
			module_put(vdev->slave[i]->module);

	kfree(fh);

	return rval;
}

/**
 * omap34xxcam_release - file operations release handler
 * @inode: ptr. to system inode structure
 * @file: ptr. to system file structure
 *
 * Complement of omap34xxcam_open.  This function will flush any scheduled
 * work, disable the sensor, close the ISP interface, stop the
 * video buffer queue from streaming and free the per-filehandle data
 * (omap34xxcam_fh).  Note that because multiple open file handles
 * are allowed, this function will only close the ISP and disable the
 * sensor when the last open file handle (by count) is closed.
 * This function returns 0.
 */
static int omap34xxcam_release(struct file *file)
{
	struct omap34xxcam_fh *fh = file->private_data;
	struct omap34xxcam_videodev *vdev = fh->vdev;
	struct device *isp = vdev->cam->isp;
	int i;

	mutex_lock(&vdev->mutex);
	if (vdev->streaming == file) {
		isp_stop(isp);
		videobuf_streamoff(&fh->vbq);
		omap34xxcam_slave_power_set(
			vdev, V4L2_POWER_STANDBY,
			OMAP34XXCAM_SLAVE_POWER_SENSOR);
		omap34xxcam_slave_power_suggest(
			vdev, V4L2_POWER_STANDBY,
			OMAP34XXCAM_SLAVE_POWER_LENS);
		vdev->streaming = NULL;
	}

	if (atomic_dec_return(&vdev->users) == 0) {
		omap34xxcam_slave_power_set(vdev, V4L2_POWER_OFF,
					    OMAP34XXCAM_SLAVE_POWER_ALL);
		isp_put();
	}
	mutex_unlock(&vdev->mutex);

	file->private_data = NULL;

	for (i = 0; i <= OMAP34XXCAM_SLAVE_FLASH; i++)
		if (vdev->slave[i] != v4l2_int_device_dummy())
			module_put(vdev->slave[i]->module);

	kfree(fh);

	return 0;
}

static struct v4l2_file_operations omap34xxcam_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.poll = omap34xxcam_poll,
	.mmap = omap34xxcam_mmap,
	.open = omap34xxcam_open,
	.release = omap34xxcam_release,
};

static void omap34xxcam_vfd_name_update(struct omap34xxcam_videodev *vdev)
{
	struct video_device *vfd = vdev->vfd;
	int i;

	strlcpy(vfd->name, CAM_SHORT_NAME, sizeof(vfd->name));
	for (i = 0; i <= OMAP34XXCAM_SLAVE_FLASH; i++) {
		strlcat(vfd->name, "/", sizeof(vfd->name));
		if (vdev->slave[i] == v4l2_int_device_dummy())
			continue;
		strlcat(vfd->name, vdev->slave[i]->name, sizeof(vfd->name));
	}
	dev_dbg(&vdev->vfd->dev, "video%d is now %s\n", vfd->num, vfd->name);
}

/**
 * omap34xxcam_device_unregister - V4L2 detach handler
 * @s: ptr. to standard V4L2 device information structure
 *
 * Detach sensor and unregister and release the video device.
 */
static void omap34xxcam_device_unregister(struct v4l2_int_device *s)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct omap34xxcam_hw_config hwc;

	BUG_ON(vidioc_int_g_priv(s, &hwc) < 0);

	mutex_lock(&vdev->mutex);

	if (vdev->slave[hwc.dev_type] != v4l2_int_device_dummy()) {
		vdev->slave[hwc.dev_type] = v4l2_int_device_dummy();
		vdev->slaves--;
		omap34xxcam_vfd_name_update(vdev);
	}

	if (vdev->slaves == 0 && vdev->vfd) {
		if (vdev->vfd->minor == -1) {
			/*
			 * The device was never registered, so release the
			 * video_device struct directly.
			 */
			video_device_release(vdev->vfd);
		} else {
			/*
			 * The unregister function will release the
			 * video_device struct as well as
			 * unregistering it.
			 */
			video_unregister_device(vdev->vfd);
		}
		vdev->vfd = NULL;
	}

	mutex_unlock(&vdev->mutex);
}

static const struct v4l2_ioctl_ops omap34xxcam_ioctl_ops = {
	.vidioc_querycap		= vidioc_querycap,
	.vidioc_enum_fmt_vid_cap	= vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= vidioc_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= vidioc_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap		= vidioc_try_fmt_vid_cap,
	.vidioc_reqbufs			= vidioc_reqbufs,
	.vidioc_querybuf		= vidioc_querybuf,
	.vidioc_qbuf			= vidioc_qbuf,
	.vidioc_dqbuf			= vidioc_dqbuf,
	.vidioc_streamon		= vidioc_streamon,
	.vidioc_streamoff		= vidioc_streamoff,
	.vidioc_enum_input		= vidioc_enum_input,
	.vidioc_g_input			= vidioc_g_input,
	.vidioc_s_input			= vidioc_s_input,
	.vidioc_queryctrl		= vidioc_queryctrl,
	.vidioc_querymenu		= vidioc_querymenu,
	.vidioc_g_ext_ctrls		= vidioc_g_ext_ctrls,
	.vidioc_s_ext_ctrls		= vidioc_s_ext_ctrls,
	.vidioc_g_parm			= vidioc_g_parm,
	.vidioc_s_parm			= vidioc_s_parm,
	.vidioc_cropcap			= vidioc_cropcap,
	.vidioc_g_crop			= vidioc_g_crop,
	.vidioc_s_crop			= vidioc_s_crop,
	.vidioc_enum_framesizes		= vidioc_enum_framesizes,
	.vidioc_enum_frameintervals	= vidioc_enum_frameintervals,
	.vidioc_s_std			= vidioc_s_std,
	.vidioc_querystd		= vidioc_querystd,
	.vidioc_default			= vidioc_default,
};

/**
 * omap34xxcam_device_register - V4L2 attach handler
 * @s: ptr. to standard V4L2 device information structure
 *
 * Allocates and initializes the V4L2 video_device structure, initializes
 * the sensor, and finally
 registers the device with V4L2 based on the
 * video_device structure.
 *
 * Returns 0 on success, otherwise an appropriate error code on
 * failure.
 */
static int omap34xxcam_device_register(struct v4l2_int_device *s)
{
	struct omap34xxcam_videodev *vdev = s->u.slave->master->priv;
	struct omap34xxcam_hw_config hwc;
	struct v4l2_ifparm ifparm;
	struct device *isp;
	int rval;

	/* We need to check rval just once. The place is here. */
	if (vidioc_int_g_priv(s, &hwc))
		return -ENODEV;

	if (vdev->index != hwc.dev_index)
		return -ENODEV;

	if (hwc.dev_type < 0 || hwc.dev_type > OMAP34XXCAM_SLAVE_FLASH)
		return -EINVAL;

	if (vdev->slave[hwc.dev_type] != v4l2_int_device_dummy())
		return -EBUSY;

	mutex_lock(&vdev->mutex);
	if (atomic_read(&vdev->users)) {
		printk(KERN_ERR "%s: we're open (%d), can't register\n",
		       __func__, atomic_read(&vdev->users));
		mutex_unlock(&vdev->mutex);
		return -EBUSY;
	}

	vdev->slaves++;

	vdev->slave[hwc.dev_type] = s;
	vdev->slave_config[hwc.dev_type] = hwc;

	if (hwc.dev_type == OMAP34XXCAM_SLAVE_SENSOR) {
		isp = isp_get();
		if (!isp) {
			rval = -EBUSY;
			printk(KERN_ERR "%s: can't get ISP, "
			       "sensor init failed\n", __func__);
			goto err;
		}
		vdev->cam->isp = isp;
	}
	rval = omap34xxcam_slave_power_set(vdev, V4L2_POWER_ON,
					   1 << hwc.dev_type);
	if (rval)
		goto err_omap34xxcam_slave_power_set;
	if (hwc.dev_type == OMAP34XXCAM_SLAVE_SENSOR) {
		struct v4l2_format format;

		format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		rval = vidioc_int_g_fmt_cap(vdev->vdev_sensor, &format);
		if (rval)
			rval = -EBUSY;

		vdev->want_pix = format.fmt.pix;
	}
	omap34xxcam_slave_power_set(vdev, V4L2_POWER_OFF, 1 << hwc.dev_type);
	if (hwc.dev_type == OMAP34XXCAM_SLAVE_SENSOR)
		isp_put();

	if (rval)
		goto err;

	/* Are we the first slave? */
	if (vdev->slaves == 1) {
		/* initialize the video_device struct */
		vdev->vfd = video_device_alloc();
		if (!vdev->vfd) {
			printk(KERN_ERR "%s: could not allocate "
			       "video device struct\n", __func__);
			rval = -ENOMEM;
			goto err;
		}
		vdev->vfd->release	= video_device_release;
		vdev->vfd->minor	= -1;
		vdev->vfd->fops		= &omap34xxcam_fops;
		vdev->vfd->ioctl_ops	= &omap34xxcam_ioctl_ops;
		video_set_drvdata(vdev->vfd, vdev);

		if (video_register_device(vdev->vfd, VFL_TYPE_GRABBER,
					  hwc.dev_minor) < 0) {
			printk(KERN_ERR "%s: could not register V4L device\n",
				__func__);
			vdev->vfd->minor = -1;
			rval = -EBUSY;
			goto err;
		}
	}
	/*Determine whether the slave connected is BT656 decoder or a sensor*/
	if (!vidioc_int_g_ifparm(s, &ifparm)) {
		if (ifparm.if_type == V4L2_IF_TYPE_BT656) {
			vdev->vfd->current_norm = V4L2_STD_NTSC;
			vdev->vfd->tvnorms	= V4L2_STD_NTSC | V4L2_STD_PAL;
			if ((ifparm.u.bt656.mode ==
					V4L2_IF_TYPE_BT656_MODE_BT_8BIT) ||
					(ifparm.u.bt656.mode ==
					 V4L2_IF_TYPE_BT656_MODE_BT_10BIT))
				vdev->slave_mode[hwc.dev_type] = 1;
		}
	}
	omap34xxcam_vfd_name_update(vdev);

	mutex_unlock(&vdev->mutex);

	return 0;

err_omap34xxcam_slave_power_set:
	if (hwc.dev_type == OMAP34XXCAM_SLAVE_SENSOR)
		isp_put();

err:
	if (s == vdev->slave[hwc.dev_type]) {
		vdev->slave[hwc.dev_type] = v4l2_int_device_dummy();
		vdev->slaves--;
	}

	mutex_unlock(&vdev->mutex);
	omap34xxcam_device_unregister(s);

	return rval;
}

static struct v4l2_int_master omap34xxcam_master = {
	.attach = omap34xxcam_device_register,
	.detach = omap34xxcam_device_unregister,
};

/*
 *
 * Module initialisation and deinitialisation
 *
 */

static void omap34xxcam_exit(void)
{
	struct omap34xxcam_device *cam = omap34xxcam;
	int i;

	if (!cam)
		return;

	for (i = 0; i < OMAP34XXCAM_VIDEODEVS; i++) {
		if (cam->vdevs[i].cam == NULL)
			continue;

		v4l2_int_device_unregister(&cam->vdevs[i].master);
		cam->vdevs[i].cam = NULL;
	}

	omap34xxcam = NULL;

	kfree(cam);
}

static int __init omap34xxcam_init(void)
{
	struct omap34xxcam_device *cam;
	int i;

	cam = kzalloc(sizeof(*cam), GFP_KERNEL);
	if (!cam) {
		printk(KERN_ERR "%s: could not allocate memory\n", __func__);
		return -ENOMEM;
	}

	omap34xxcam = cam;

	for (i = 0; i < OMAP34XXCAM_VIDEODEVS; i++) {
		struct omap34xxcam_videodev *vdev = &cam->vdevs[i];
		struct v4l2_int_device *m = &vdev->master;

		m->module       = THIS_MODULE;
		strlcpy(m->name, CAM_NAME, sizeof(m->name));
		m->type         = v4l2_int_type_master;
		m->u.master     = &omap34xxcam_master;
		m->priv		= vdev;

		mutex_init(&vdev->mutex);
		vdev->index             = i;
		vdev->cam               = cam;
		vdev->vdev_sensor =
			vdev->vdev_lens =
			vdev->vdev_flash = v4l2_int_device_dummy();
#ifdef OMAP34XXCAM_POWEROFF_DELAY
		setup_timer(&vdev->poweroff_timer,
			    omap34xxcam_slave_power_timer, (unsigned long)vdev);
		INIT_WORK(&vdev->poweroff_work, omap34xxcam_slave_power_work);
#endif /* OMAP34XXCAM_POWEROFF_DELAY */

		if (v4l2_int_device_register(m))
			goto err;
	}

	return 0;

err:
	omap34xxcam_exit();
	return -ENODEV;
}

MODULE_AUTHOR("Sakari Ailus <sakari.ailus@nokia.com>");
MODULE_DESCRIPTION("OMAP34xx Video for Linux camera driver");
MODULE_LICENSE("GPL");

late_initcall(omap34xxcam_init);
module_exit(omap34xxcam_exit);
