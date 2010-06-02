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
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <asm/pgalloc.h>

#include <media/v4l2-common.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#include "omap34xxcam.h"
#include "isp/ispvideo.h"
#include "isp/isp.h"

/* -----------------------------------------------------------------------------
 * V4L2 ioctl operations
 */

/* List of image formats supported via OMAP ISP */
static const struct v4l2_fmtdesc isp_formats[] = {
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
 * vidioc_enum_fmt_vid_cap - V4L2 enumerate format capabilities IOCTL handler
 * @file: ptr. to system file structure
 * @f: ptr to standard V4L2 format description structure
 *
 * Fills in enumerate format capabilities information for ISP.
 */
static int vidioc_enum_fmt_vid_cap(struct file *file, void *fh,
				   struct v4l2_fmtdesc *f)
{
	int index = f->index;
	enum v4l2_buf_type type = f->type;
	int rval = -EINVAL;

	if (index >= ARRAY_SIZE(isp_formats))
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

/*
 * omap34xxcam_set_format - Set formats at all pads in the ISP pipeline.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @pix_input: Pixel format to use as input in the ISP.
 * @pix_output: Pixel format output by the ISP to fill back.
 * @which: Select the probe or active format.
 *
 * Returns the closest possible output size based on silicon limitations
 * detailed through the pipe structure.
 *
 * Return or if successful or a negative error code otherwise. Possible errors
 * are:
 *
 * -EINVAL	The input image size isn't supported by the CCDC
 */
static int
omap34xxcam_set_format(struct omap34xxcam_videodev *vdev,
		       const struct v4l2_pix_format *pix_input,
		       struct v4l2_pix_format *pix_output,
		       enum v4l2_subdev_format which)
{
	unsigned int wanted_width = pix_output->width;
	unsigned int wanted_height = pix_output->height;
	enum v4l2_mbus_pixelcode pixelcode;
	struct v4l2_mbus_framefmt fmt;
	struct isp_device *isp = vdev->cam->isp;

	isp_video_pix_to_mbus(pix_output, &fmt);
	pixelcode = fmt.code;

	isp_video_pix_to_mbus(pix_input, &fmt);

	/* Set sensor output format */
	v4l2_subdev_call(vdev->vdev_sensor, pad, set_fmt, 0, &fmt, which);

	if (vdev->vdev_sensor->entity.links[0].sink->entity ==
	   &isp->isp_ccp2.subdev.entity) {
		/* Set format for CCP2 */
		v4l2_subdev_call(&isp->isp_ccp2.subdev, pad, set_fmt,
			 CCP2_PAD_SINK, &fmt, which);

		v4l2_subdev_call(&isp->isp_ccp2.subdev, pad, set_fmt,
			 CCP2_PAD_SOURCE, &fmt, which);
	} else {
		/* Set format for CSI2 */
		v4l2_subdev_call(&isp->isp_csi2a.subdev, pad, set_fmt,
			 CSI2_PAD_SINK, &fmt, which);

		/* Currently output from CSI2 is always
		   V4L2_MBUS_FMT_SGRBG10_1X10 */
		fmt.code = V4L2_MBUS_FMT_SGRBG10_1X10;
		v4l2_subdev_call(&isp->isp_csi2a.subdev, pad, set_fmt,
			 CSI2_PAD_SOURCE, &fmt, which);
	}

	/* Set the format at the CCDC input. */
	isp_video_pix_to_mbus(pix_input, &fmt);

	v4l2_subdev_call(&isp->isp_ccdc.subdev, pad, set_fmt,
			 CCDC_PAD_SINK, &fmt, which);

	if (fmt.width != pix_input->width || fmt.height != pix_input->height) {
		dev_dbg(isp->dev, "input image size %ux%u not "
			"supported by the CCDC\n",
			pix_input->width, pix_input->height);
		return -EINVAL;
	}

	if (pix_output->pixelformat == V4L2_PIX_FMT_SGRBG10) {
		/* When capturing the CCDC output to memory, request a
		 * number of horizontal pixel bigger than the input as
		 * the CCDC will clip it to a multiple of 16.
		 */
		fmt.width = (fmt.width + 15) & ~15;

		v4l2_subdev_call(&isp->isp_ccdc.subdev, pad, set_fmt,
				 CCDC_PAD_SOURCE_OF, &fmt, which);

		isp_video_mbus_to_pix(&isp->isp_ccdc.video_out, &fmt,
				      pix_output);
	}

	fmt.height--;

	v4l2_subdev_call(&isp->isp_ccdc.subdev, pad, set_fmt,
			 CCDC_PAD_SOURCE_VP, &fmt, which);

	if (pix_output->pixelformat != V4L2_PIX_FMT_SGRBG10) {

		/* Set the format at the preview module input. */
		v4l2_subdev_call(&isp->isp_prev.subdev, pad, set_fmt,
				 PREV_PAD_SINK, &fmt, which);

		/* Set and retrieve the format at the preview module output. */
		fmt.code = pixelcode;

		v4l2_subdev_call(&isp->isp_prev.subdev, pad, set_fmt,
				 PREV_PAD_SOURCE, &fmt, which);

		isp_video_mbus_to_pix(&isp->isp_prev.video_out, &fmt,
				      pix_output);

		/* Set the format at the resizer module input. */
		v4l2_subdev_call(&isp->isp_res.subdev, pad, set_fmt,
				 RESZ_PAD_SINK, &fmt, which);

		/* Set and retrieve the format at the resizer module output. */
		fmt.width = wanted_width;
		fmt.height = wanted_height;

		v4l2_subdev_call(&isp->isp_res.subdev, pad, set_fmt,
				 RESZ_PAD_SOURCE, &fmt, which);

		isp_video_mbus_to_pix(&isp->isp_res.video_out, &fmt,
				      pix_output);
	}

	return 0;
}

static inline unsigned int mbus_code_to_pixfmt(unsigned int code)
{
	switch (code) {
	case V4L2_MBUS_FMT_SGRBG10_1X10:
		return V4L2_PIX_FMT_SGRBG10;
	case V4L2_MBUS_FMT_SGRBG10_DPCM8_1X8:
		return V4L2_PIX_FMT_SGRBG10DPCM8;
	case V4L2_MBUS_FMT_YUYV16_1X16:
		return V4L2_PIX_FMT_YUYV;
	case V4L2_MBUS_FMT_UYVY16_1X16:
	default:
		return V4L2_PIX_FMT_UYVY;
	}
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
	struct omap34xxcam_device *cam = vdev->cam;

	if (best_ival->numerator == 0 || best_ival->denominator == 0)
		*best_ival = cam->pdata->sensors[vdev->index].ival_default;

	fps = best_ival->denominator / best_ival->numerator;

	memset(best_pix_in, 0, sizeof(*best_pix_in));

	best_ival->denominator = 0;
	best_pix_out.height = INT_MAX >> 1;
	best_pix_out.width = best_pix_out.height;

	for (fmtd_index = 0; ; fmtd_index++) {
		int size_index;
		struct v4l2_subdev_pad_mbus_code_enum code;

		code.index = fmtd_index;
		rval = v4l2_subdev_call(vdev->vdev_sensor, pad,
					enum_mbus_code, &code);
		if (rval)
			break;
		dev_dbg(&vdev->video.video.dev, "trying fmt %8.8x (%d)\n",
			code.code, fmtd_index);
		/*
		 * Get supported resolutions.
		 */
		for (size_index = 0; ; size_index++) {
			struct v4l2_frmsizeenum frms;
			struct v4l2_pix_format pix_tmp_in, pix_tmp_out;
			int ival_index;

			frms.index = size_index;
			frms.pixel_format = mbus_code_to_pixfmt(code.code);

			rval = v4l2_subdev_call(vdev->vdev_sensor, video,
						enum_framesizes, &frms);
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
			rval = omap34xxcam_set_format(vdev, &pix_tmp_in,
						      &pix_tmp_out,
						      V4L2_SUBDEV_FORMAT_PROBE);
			if (rval)
				return rval;

			dev_dbg(&vdev->video.video.dev,
				"this w %d\th %d\tfmt %8.8x\t"
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
				dev_dbg(&vdev->video.video.dev,
					"size diff bigger: "
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
				dev_dbg(&vdev->video.video.dev, "renegotiate: "
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

				rval = v4l2_subdev_call(vdev->vdev_sensor,
					video, enum_frameintervals, &frmi);
				if (rval)
					break;

				dev_dbg(&vdev->video.video.dev, "fps %d\n",
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
					dev_dbg(&vdev->video.video.dev,
						"closer fps: "
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
					dev_dbg(&vdev->video.video.dev,
						"bigger res, "
						"same fps: "
						"w %d\th %d\tw %d\th %d\n",
						frmi.width, frmi.height,
						best_pix_in->width,
						best_pix_in->height);
					goto do_it_now;
				}

				dev_dbg(&vdev->video.video.dev,
					"falling through\n");

				continue;

do_it_now:
				*best_ival = frmi.discrete;
				best_pix_out = pix_tmp_out;
				best_pix_in->width = frmi.width;
				best_pix_in->height = frmi.height;
				best_pix_in->pixelformat = frmi.pixel_format;

				dev_dbg(&vdev->video.video.dev,
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

	dev_dbg(&vdev->video.video.dev, "w %d, h %d, fmt %8.8x -> w %d, h %d\n",
		best_pix_in->width, best_pix_in->height,
		best_pix_in->pixelformat,
		best_pix_out.width, best_pix_out.height);

	return 0;
}

static int s_pix_parm(struct omap34xxcam_videodev *vdev,
		      struct v4l2_pix_format *pix,
		      struct v4l2_fract *best_ival)
{
	struct isp_video *video = &vdev->video;
	struct isp_device *isp = vdev->cam->isp;
	struct media_entity_pad *remote;
	struct media_entity_pad *source;
	struct media_entity_pad *sink;
	struct media_entity_link *link;
	struct v4l2_pix_format best_pix;
	struct v4l2_subdev_frame_interval ival;
	int rval;

	/*
	 * Setup pipeline links.
	 */

	/*
	 * Activate the CCDC -> Preview, Preview -> Resizer and
	 * Resizer -> memory links when capturing YUV video.
	 */
	source = &isp->isp_ccdc.subdev.entity.pads[CCDC_PAD_SOURCE_VP];
	sink = &isp->isp_prev.subdev.entity.pads[PREV_PAD_SINK];
	link = media_entity_find_link(source, sink);
	rval = media_entity_setup_link(link,
			pix->pixelformat == V4L2_PIX_FMT_SGRBG10 ?
			0 : MEDIA_LINK_FLAG_ACTIVE);
	if (rval < 0) {
		dev_info(isp->dev, "unable to setup CCDC -> Preview link.\n");
		return rval;
	}

	source = &isp->isp_prev.subdev.entity.pads[PREV_PAD_SOURCE];
	sink = &isp->isp_res.subdev.entity.pads[RESZ_PAD_SINK];
	link = media_entity_find_link(source, sink);
	rval = media_entity_setup_link(link,
			pix->pixelformat == V4L2_PIX_FMT_SGRBG10 ?
			0 : MEDIA_LINK_FLAG_ACTIVE);
	if (rval < 0) {
		dev_info(isp->dev,
			 "unable to setup Preview -> Resizer link.\n");
		return rval;
	}

	/* Activate the correct video device link. */
	if (pix->pixelformat == V4L2_PIX_FMT_SGRBG10)
		source = &isp->isp_ccdc.subdev.entity.pads[CCDC_PAD_SOURCE_OF];
	else
		source = &isp->isp_res.subdev.entity.pads[RESZ_PAD_SOURCE];

	remote = media_entity_remote_pad(&video->pad);

	if (source != remote) {
		if (remote != NULL) {
			sink = &video->video.entity.pads[0];
			link = media_entity_find_link(remote, sink);
			rval = media_entity_setup_link(link, 0);
			if (rval < 0) {
				dev_err(&vdev->video.video.dev,
					"can't disconnect video device node\n");
				return rval;
			}
		}

		sink = &video->video.entity.pads[0];
		link = media_entity_find_link(source, sink);
		rval = media_entity_setup_link(link, MEDIA_LINK_FLAG_ACTIVE);
		if (rval < 0) {
			dev_err(&vdev->video.video.dev,
				"can't connect video device node\n");
			return rval;
		}
	}

	rval = try_pix_parm(vdev, &best_pix, pix, best_ival);
	if (rval)
		return rval;

	rval = omap34xxcam_set_format(vdev, &best_pix, pix,
				      V4L2_SUBDEV_FORMAT_ACTIVE);
	if (rval)
		return rval;

	memset(&ival, 0, sizeof(ival));
	ival.interval = *best_ival;
	return v4l2_subdev_call(vdev->vdev_sensor, video, s_frame_interval,
				&ival);
}

/**
 * vidioc_s_fmt_vid_cap - V4L2 set format capabilities IOCTL handler
 * @file: ptr. to system file structure
 * @f: ptr to standard V4L2 format structure
 *
 * Attempts to set input format with the sensor driver (first) and then the
 * ISP.  Returns the return code from vidioc_g_fmt_vid_cap().
 */
static int vidioc_s_fmt_vid_cap(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct isp_video *video = video_drvdata(file);
	struct omap34xxcam_videodev *vdev = to_omap34xxcam_videodev(video);
	struct v4l2_fract timeperframe;
	int rval;

	mutex_lock(&vdev->mutex);
	if (vdev->streaming) {
		rval = -EBUSY;
		goto out;
	}

	vdev->want_pix = f->fmt.pix;
	timeperframe = vdev->want_timeperframe;
	rval = s_pix_parm(vdev, &f->fmt.pix, &timeperframe);

out:
	mutex_unlock(&vdev->mutex);
	return rval;
}

/**
 * vidioc_try_fmt_vid_cap - V4L2 try format capabilities IOCTL handler
 * @file: ptr. to system file structure
 * @f: ptr to standard V4L2 format structure
 *
 * Checks if the given format is supported by the sensor driver and
 * by the ISP.
 */
static int vidioc_try_fmt_vid_cap(struct file *file, void *fh,
				  struct v4l2_format *f)
{
	struct isp_video *video = video_drvdata(file);
	struct omap34xxcam_videodev *vdev = to_omap34xxcam_videodev(video);
	struct v4l2_pix_format pix_tmp;
	struct v4l2_fract timeperframe;
	int rval;

	mutex_lock(&vdev->mutex);

	timeperframe = vdev->want_timeperframe;

	rval = try_pix_parm(vdev, &pix_tmp, &f->fmt.pix, &timeperframe);

	mutex_unlock(&vdev->mutex);

	return rval;
}

/**
 * vidioc_queryctrl - V4L2 query control IOCTL handler
 * @file: ptr. to system file structure
 * @a: standard V4L2 query control ioctl structure
 *
 * If the requested control is supported, returns the control information
 * in the v4l2_queryctrl structure.  Otherwise, returns -EINVAL if the
 * control is not supported. The ISP is queried and if it does not support
 * the requested control, the request is forwarded to the sensor driver to
 * see if it supports it.
 */
static int vidioc_queryctrl(struct file *file, void *fh,
			    struct v4l2_queryctrl *query)
{
	struct isp_video *video = video_drvdata(file);
	struct omap34xxcam_videodev *vdev = to_omap34xxcam_videodev(video);
	struct v4l2_queryctrl best;
	struct v4l2_subdev *sd;
	int ret = -EINVAL;

	/* No next flags: try subdevs directly. */
	if (!(query->id & V4L2_CTRL_FLAG_NEXT_CTRL)) {
		v4l2_device_for_each_subdev(sd, &vdev->cam->v4l2_dev) {
			if (sd->grp_id != vdev->index &&
			    sd->grp_id != (1 << 16))
				continue;

			ret = v4l2_subdev_call(sd, core, queryctrl, query);
			if (ret != -ENOIOCTLCMD && ret != -EINVAL)
				break;
		}

		return ret == -ENOIOCTLCMD ? -EINVAL : ret;
	}

	/* Find subdev with smallest next control id. */
	best.id = (u32)-1;

	v4l2_device_for_each_subdev(sd, &vdev->cam->v4l2_dev) {
		struct v4l2_queryctrl __query;

		if (sd->grp_id != vdev->index &&
		    sd->grp_id != (1 << 16))
			continue;

		__query = *query;
		ret = v4l2_subdev_call(sd, core, queryctrl, &__query);
		if (ret == 0 && __query.id < best.id)
			best = __query;
	}

	if (best.id == (u32)-1)
		return -EINVAL;

	*query = best;
	return 0;
}

/**
 * vidioc_querymenu - V4L2 query menu IOCTL handler
 * @file: ptr. to system file structure
 * @a: standard V4L2 query menu ioctl structure
 *
 * If the requested control is supported, returns the menu information
 * in the v4l2_querymenu structure.  Otherwise, returns -EINVAL if the
 * control is not supported or is not a menu. The ISP is queried and if
 * it does not support the requested menu control, the request is forwarded
 * to the "raw" sensor driver to see if it supports it.
 */
static int vidioc_querymenu(struct file *file, void *fh,
			    struct v4l2_querymenu *query)
{
	struct isp_video *video = video_drvdata(file);
	struct omap34xxcam_videodev *vdev = to_omap34xxcam_videodev(video);
	struct v4l2_subdev *sd;
	int ret = -EINVAL;

	v4l2_device_for_each_subdev(sd, &vdev->cam->v4l2_dev) {
		if (sd->grp_id != vdev->index &&
		    sd->grp_id != (1 << 16))
			continue;

		ret = v4l2_subdev_call(sd, core, querymenu, query);
		if (ret != -ENOIOCTLCMD && ret != -EINVAL)
			break;
	}

	return ret == -ENOIOCTLCMD ? -EINVAL : ret;
}

static int vidioc_g_ext_ctrls(struct file *file, void *fh,
			      struct v4l2_ext_controls *ctrls)
{
	struct isp_video *video = video_drvdata(file);
	struct omap34xxcam_videodev *vdev = to_omap34xxcam_videodev(video);
	unsigned int i;
	int ret = 0;

	mutex_lock(&vdev->mutex);

	for (i = 0; i < ctrls->count; i++) {
		struct v4l2_control ctrl;
		struct v4l2_subdev *sd;

		ctrl.id = ctrls->controls[i].id;
		ret = -EINVAL;

		v4l2_device_for_each_subdev(sd, &vdev->cam->v4l2_dev) {
			if (sd->grp_id != vdev->index &&
			    sd->grp_id != (1 << 16))
				continue;

			ret = v4l2_subdev_call(sd, core, g_ctrl, &ctrl);
			if (ret != -ENOIOCTLCMD && ret != -EINVAL)
				break;
		}

		if (ret == -ENOIOCTLCMD)
			ret = -EINVAL;

		if (ret < 0) {
			ctrls->error_idx = i;
			break;
		}

		ctrls->controls[i].value = ctrl.value;
	}

	mutex_unlock(&vdev->mutex);

	return ret;
}

static int vidioc_s_ext_ctrls(struct file *file, void *fh,
			      struct v4l2_ext_controls *ctrls)
{
	struct isp_video *video = video_drvdata(file);
	struct omap34xxcam_videodev *vdev = to_omap34xxcam_videodev(video);
	unsigned int i;
	int ret = 0;

	mutex_lock(&vdev->mutex);

	for (i = 0; i < ctrls->count; i++) {
		struct v4l2_control ctrl;
		struct v4l2_subdev *sd;

		ctrl.id = ctrls->controls[i].id;
		ctrl.value = ctrls->controls[i].value;
		ret = -EINVAL;

		v4l2_device_for_each_subdev(sd, &vdev->cam->v4l2_dev) {
			if (sd->grp_id != vdev->index &&
			    sd->grp_id != (1 << 16))
				continue;

			ret = v4l2_subdev_call(sd, core, s_ctrl, &ctrl);
			if (ret != -ENOIOCTLCMD && ret != -EINVAL)
				break;
		}

		if (ret == -ENOIOCTLCMD)
			ret = -EINVAL;

		if (ret < 0) {
			ctrls->error_idx = i;
			break;
		}

		ctrls->controls[i].value = ctrl.value;
	}

	mutex_unlock(&vdev->mutex);

	return ret;
}

/**
 * vidioc_g_parm - V4L2 get parameters IOCTL handler
 * @file: ptr. to system file structure
 * @a: standard V4L2 stream parameters structure
 *
 * If request is for video capture buffer type, handles request by
 * forwarding to sensor driver.
 */
static int vidioc_g_parm(struct file *file, void *fh, struct v4l2_streamparm *a)
{
	struct isp_video *video = video_drvdata(file);
	struct omap34xxcam_videodev *vdev = to_omap34xxcam_videodev(video);
	struct v4l2_subdev_frame_interval ival;
	int rval;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	mutex_lock(&vdev->mutex);
	rval = v4l2_subdev_call(vdev->vdev_sensor, video, g_frame_interval,
				&ival);
	mutex_unlock(&vdev->mutex);

	a->parm.capture.timeperframe = ival.interval;
	return rval;
}

/**
 * vidioc_s_parm - V4L2 set parameters IOCTL handler
 * @file: ptr. to system file structure
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
	struct isp_video *video = video_drvdata(file);
	struct omap34xxcam_videodev *vdev = to_omap34xxcam_videodev(video);
	struct v4l2_pix_format pix_tmp;
	int rval;

	if (a->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	mutex_lock(&vdev->mutex);
	if (vdev->streaming) {
		rval = -EBUSY;
		goto out;
	}

	vdev->want_timeperframe = a->parm.capture.timeperframe;

	pix_tmp = vdev->want_pix;

	rval = s_pix_parm(vdev, &pix_tmp, &a->parm.capture.timeperframe);

out:
	mutex_unlock(&vdev->mutex);

	return rval;
}

static int vidioc_enum_framesizes(struct file *file, void *fh,
				  struct v4l2_frmsizeenum *frms)
{
	struct isp_video *video = video_drvdata(file);
	struct omap34xxcam_videodev *vdev = to_omap34xxcam_videodev(video);
	struct v4l2_pix_format pix_in;
	struct v4l2_pix_format pix_out;
	struct v4l2_fract ival;
	u32 pixel_format;
	int rval;

	mutex_lock(&vdev->mutex);

	pixel_format = frms->pixel_format;
	frms->pixel_format = -1;	/* ISP does format conversion */
	rval = v4l2_subdev_call(vdev->vdev_sensor, video, enum_framesizes,
				frms);
	frms->pixel_format = pixel_format;

	if (rval < 0)
		goto done;

	/* Let the ISP pipeline mangle the frame size as it sees fit. */
	memset(&pix_out, 0, sizeof(pix_out));
	pix_out.width = frms->discrete.width;
	pix_out.height = frms->discrete.height;
	pix_out.pixelformat = frms->pixel_format;

	ival = vdev->want_timeperframe;
	rval = try_pix_parm(vdev, &pix_in, &pix_out, &ival);
	if (rval < 0)
		goto done;

	frms->discrete.width = pix_out.width;
	frms->discrete.height = pix_out.height;

done:
	mutex_unlock(&vdev->mutex);
	return rval;
}

static int vidioc_enum_frameintervals(struct file *file, void *fh,
				      struct v4l2_frmivalenum *frmi)
{
	struct isp_video *video = video_drvdata(file);
	struct omap34xxcam_videodev *vdev = to_omap34xxcam_videodev(video);
	struct v4l2_frmsizeenum frms;
	unsigned int frmi_width;
	unsigned int frmi_height;
	unsigned int uninitialized_var(width);
	unsigned int uninitialized_var(height);
	unsigned int max_dist;
	unsigned int dist;
	u32 pixel_format;
	unsigned int i;
	int rval;

	mutex_lock(&vdev->mutex);

	/*
	 * Frame size enumeration returned sizes mangled by the ISP.
	 * We can't pass the size directly to the sensor for frame
	 * interval enumeration, as they will not be recognized by the
	 * sensor driver. Enumerate the native sensor sizes and select
	 * the one closest to the requested size.
	 */

	for (i = 0, max_dist = (unsigned int)-1; ; ++i) {
		frms.index = i;
		frms.pixel_format = -1;
		rval = v4l2_subdev_call(vdev->vdev_sensor, video,
					enum_framesizes, &frms);
		if (rval < 0)
			break;

		/*
		 * The distance between frame sizes is the size in
		 * pixels of the non-overlapping regions.
		 */
		dist = min(frms.discrete.width, frmi->width)
		     * min(frms.discrete.height, frmi->height);
		dist = frms.discrete.width * frms.discrete.height
		     + frmi->width * frmi->height
		     - 2*dist;

		if (dist < max_dist) {
			width = frms.discrete.width;
			height = frms.discrete.height;
			max_dist = dist;
		}
	}

	if (max_dist == (unsigned int)-1) {
		rval = -EINVAL;
		goto done;
	}

	pixel_format = frmi->pixel_format;
	frmi_width = frmi->width;
	frmi_height = frmi->height;

	frmi->pixel_format = -1;	/* ISP does format conversion */
	frmi->width = width;
	frmi->height = height;
	rval = v4l2_subdev_call(vdev->vdev_sensor, video, enum_frameintervals,
				frmi);

	frmi->pixel_format = pixel_format;
	frmi->height = frmi_height;
	frmi->width = frmi_width;

done:
	mutex_unlock(&vdev->mutex);
	return rval;
}

static const struct v4l2_ioctl_ops omap34xxcam_ioctl_ops = {
	.vidioc_enum_fmt_vid_cap	= vidioc_enum_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= vidioc_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap		= vidioc_try_fmt_vid_cap,
	.vidioc_queryctrl		= vidioc_queryctrl,
	.vidioc_querymenu		= vidioc_querymenu,
	.vidioc_g_ext_ctrls		= vidioc_g_ext_ctrls,
	.vidioc_s_ext_ctrls		= vidioc_s_ext_ctrls,
	.vidioc_g_parm			= vidioc_g_parm,
	.vidioc_s_parm			= vidioc_s_parm,
	.vidioc_enum_framesizes		= vidioc_enum_framesizes,
	.vidioc_enum_frameintervals	= vidioc_enum_frameintervals,
};

/* -----------------------------------------------------------------------------
 * ISP video operations
 */

static int
omap34xxcam_video_init(struct isp_video *video)
{
	struct omap34xxcam_videodev *vdev = to_omap34xxcam_videodev(video);
	struct isp_device *isp = video->isp;
	struct media_entity_pad *source;
	struct media_entity_pad *sink;
	struct media_entity_link *link;
	struct v4l2_fract timeperframe;
	struct v4l2_mbus_framefmt format;
	struct v4l2_pix_format pixfmt;
	int ret;

	if (atomic_inc_return(&vdev->cam->refcount) > 1) {
		atomic_dec(&vdev->cam->refcount);
		return -EBUSY;
	}

	mutex_lock(&vdev->mutex);

	/* Activate the CSI2a -> CCDC (primary sensor) or CCP2 -> CCDC
	 * (secondary sensor) link.
	 */
	if (vdev->vdev_sensor->entity.links[0].sink->entity ==
	    &isp->isp_csi2a.subdev.entity) {
		source = &isp->isp_csi2a.subdev.entity.pads[CSI2_PAD_SOURCE];
		sink = &isp->isp_ccdc.subdev.entity.pads[CCDC_PAD_SINK];
		link = media_entity_find_link(source, sink);
		ret = media_entity_setup_link(link, MEDIA_LINK_FLAG_ACTIVE);
		if (ret < 0) {
			dev_err(&vdev->video.video.dev,
				"can't connect CSI2a to CCDC\n");
			goto done;
		}
	} else {
		/* Activate the sensor -> CCP2 link */
		source = &vdev->vdev_sensor->entity.pads[0];
		sink = &video->isp->isp_ccp2.subdev.entity.pads[CCP2_PAD_SINK];
		link = media_entity_find_link(source, sink);
		ret = media_entity_setup_link(link, MEDIA_LINK_FLAG_ACTIVE);
		if (ret < 0) {
			dev_err(&vdev->video.video.dev,
				"can't connect sensor to CCP2\n");
			goto done;
		}

		/* Activate the CCP2 -> CCDC link. */
		source =
		     &video->isp->isp_ccp2.subdev.entity.pads[CCP2_PAD_SOURCE];
		sink = &video->isp->isp_ccdc.subdev.entity.pads[CCDC_PAD_SINK];
		link = media_entity_find_link(source, sink);
		ret = media_entity_setup_link(link, MEDIA_LINK_FLAG_ACTIVE);
		if (ret < 0) {
			dev_err(&vdev->video.video.dev,
				"can't connect CCP2 to CCDC\n");
			goto done;
		}
	}

	/* Get the format the sensor is using. */
	ret = v4l2_subdev_call(vdev->vdev_sensor, pad, get_fmt, 0, &format,
			       V4L2_SUBDEV_FORMAT_ACTIVE);
	if (ret) {
		dev_err(&vdev->video.video.dev,
			"can't get current sensor output format!\n");
		goto done;
	}

	pixfmt.width = format.width;
	pixfmt.height = format.height;
	pixfmt.pixelformat = V4L2_PIX_FMT_SGRBG10;

	timeperframe = vdev->want_timeperframe;

	ret = s_pix_parm(vdev, &pixfmt, &timeperframe);
	if (ret) {
		dev_err(&vdev->video.video.dev,
			"isp doesn't like the sensor!\n");
		goto done;
	}

done:
	if (ret < 0) {
		atomic_dec(&vdev->cam->refcount);
	}

	mutex_unlock(&vdev->mutex);
	return ret;

}

static int
omap34xxcam_video_cleanup(struct isp_video *video)
{
	struct omap34xxcam_videodev *vdev = to_omap34xxcam_videodev(video);
	struct isp_device *isp = video->isp;
	struct media_entity_pad *source;
	struct media_entity_pad *sink;
	struct media_entity_link *link;
	int ret;

	mutex_lock(&vdev->mutex);

	/* Deactivate the CSI2a -> CCDC (primary sensor) or the sensor -> CCDC
	 * (secondary sensor) link.
	 */
	if (vdev->vdev_sensor->entity.links[0].sink->entity ==
	    &isp->isp_csi2a.subdev.entity) {
		source = &isp->isp_csi2a.subdev.entity.pads[CSI2_PAD_SOURCE];
		sink = &isp->isp_ccdc.subdev.entity.pads[CCDC_PAD_SINK];
		link = media_entity_find_link(source, sink);
		ret = media_entity_setup_link(link, 0);
		if (ret < 0)
			dev_err(&vdev->video.video.dev,
				"can't disconnect CSI2a from CCDC\n");
	} else {
		/* Deactivate the sensor -> CCP2 link */
		source = &vdev->vdev_sensor->entity.pads[0];
		sink = &video->isp->isp_ccp2.subdev.entity.pads[CCP2_PAD_SINK];
		link = media_entity_find_link(source, sink);
		ret = media_entity_setup_link(link, 0);
		if (ret < 0)
			dev_err(&vdev->video.video.dev,
				"can't disconnect sensor from CCP2\n");

		source = &isp->isp_ccp2.subdev.entity.pads[CCP2_PAD_SOURCE];
		sink = &isp->isp_ccdc.subdev.entity.pads[CCDC_PAD_SINK];
		link = media_entity_find_link(source, sink);
		ret = media_entity_setup_link(link, 0);
		if (ret < 0)
			dev_err(&vdev->video.video.dev,
				"can't disconnect CCP2 from CCDC\n");
	}

	/* Deactivate the CCDC/resizer -> video device node link. */
	source = media_entity_remote_pad(&video->pad);
	if (source != NULL) {
		sink = &video->video.entity.pads[0];
		link = media_entity_find_link(source, sink);
		ret = media_entity_setup_link(link, 0);
		if (ret < 0)
			dev_err(&vdev->video.video.dev,
				"can't disconnect video device node\n");
	}

	mutex_unlock(&vdev->mutex);

	atomic_dec(&vdev->cam->refcount);
	return 0;
}

static int
omap34xxcam_video_stream_off(struct isp_video *video)
{
	isp_stop(video->isp, video);
	return 0;
}

static int
omap34xxcam_video_queue(struct isp_video *video, struct isp_buffer *buffer)
{
	struct isp_device *isp = video->isp;

	if (isp->isp_ccdc.output & CCDC_OUTPUT_PREVIEW)
		isp->isp_res.video_out.ops->queue(&isp->isp_res.video_out,
						  buffer);
	else if (isp->isp_ccdc.output & CCDC_OUTPUT_MEMORY)
		isp->isp_ccdc.video_out.ops->queue(&isp->isp_ccdc.video_out,
						   buffer);
	return 0;
}

static const struct isp_video_operations omap34xxcam_video_ops = {
	.init = omap34xxcam_video_init,
	.cleanup = omap34xxcam_video_cleanup,
	.stream_off = omap34xxcam_video_stream_off,
	.queue = omap34xxcam_video_queue,
};

/* -----------------------------------------------------------------------------
 * File operations
 */

static void omap34xxcam_name_update(struct omap34xxcam_videodev *vdev)
{
	unsigned int i;

	strlcpy(vdev->name, CAM_SHORT_NAME, sizeof(vdev->name));

	for (i = 0; i <= OMAP34XXCAM_SUBDEV_FLASH; i++) {
		strlcat(vdev->name, "/", sizeof(vdev->name));
		if (vdev->subdev[i] != NULL)
			strlcat(vdev->name, vdev->subdev[i]->name,
				sizeof(vdev->name));
	}
}

/**
 * omap34xxcam_device_unregister - V4L2 detach handler
 * @s: ptr. to standard V4L2 device information structure
 *
 * Detach sensor and unregister and release the video device.
 */
static void omap34xxcam_device_unregister(struct omap34xxcam_videodev *vdev)
{
	mutex_lock(&vdev->mutex);
	isp_video_unregister(&vdev->video);
	mutex_unlock(&vdev->mutex);
}

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
static int
omap34xxcam_device_register(struct omap34xxcam_videodev *vdev,
			    struct v4l2_subdev_i2c_board_info *board_info)
{
	struct v4l2_mbus_framefmt format;
	int rval;

	if (board_info->board_info == NULL)
		return -ENODEV;

	mutex_lock(&vdev->mutex);

	for (; board_info->board_info; ++board_info) {
		struct v4l2_subdev *subdev;
		struct i2c_adapter *adapter;

		adapter = i2c_get_adapter(board_info->i2c_adapter_id);
		if (adapter == NULL) {
			printk(KERN_ERR "%s: Unable to get I2C adapter %d for "
				"device %s\n", __func__,
				board_info->i2c_adapter_id,
				board_info->board_info->type);
			rval = -EINVAL;
			goto err;
		}

		subdev = v4l2_i2c_new_subdev_board(&vdev->cam->v4l2_dev,
				adapter, board_info->module_name,
				board_info->board_info, NULL);
		if (subdev == NULL) {
			printk(KERN_ERR "%s: Unable to register subdev %s\n",
				__func__, board_info->board_info->type);
			rval = -EINVAL;
			goto err;
		}

		subdev->grp_id = vdev->index;
		vdev->subdev[vdev->subdevs++] = subdev;
	}

	rval = v4l2_subdev_call(vdev->vdev_sensor, pad, get_fmt, 0, &format,
				V4L2_SUBDEV_FORMAT_ACTIVE);

	if (rval) {
		rval = -EBUSY;
		goto err;
	}

	vdev->want_pix.width = format.width;
	vdev->want_pix.height = format.height;
	vdev->want_pix.pixelformat = V4L2_PIX_FMT_SGRBG10;

	/* initialize the video_device struct */
	omap34xxcam_name_update(vdev);

	vdev->video.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vdev->video.ops = &omap34xxcam_video_ops;
	vdev->video.ioctl_ops = &omap34xxcam_ioctl_ops;
	vdev->video.isp = vdev->cam->isp;
	vdev->video.capture_mem =
		vdev->cam->pdata->sensors[vdev->index].capture_mem;
	vdev->video.alignment = 32;

	rval = isp_video_init(&vdev->video, vdev->name);
	if (rval < 0) {
		printk(KERN_ERR "%s: could not initialize video device (%d)\n",
			__func__, rval);
		goto err;
	}

	strlcpy(vdev->video.video.name, vdev->name,
		sizeof(vdev->video.video.name));
	mutex_unlock(&vdev->mutex);
	return 0;

err:
	mutex_unlock(&vdev->mutex);
	omap34xxcam_device_unregister(vdev);
	return rval;
}

static int omap34xxcam_remove(struct platform_device *pdev)
{
	struct omap34xxcam_platform_data *pdata = pdev->dev.platform_data;
	struct v4l2_device *vdev = platform_get_drvdata(pdev);
	struct omap34xxcam_device *cam = to_omap34xxcam_device(vdev);
	int i;

	omap3isp_unregister_entities(pdata->isp);

	for (i = 0; i < OMAP34XXCAM_VIDEODEVS; i++) {
		if (cam->vdevs[i].cam == NULL)
			continue;

		omap34xxcam_device_unregister(&cam->vdevs[i]);

		cam->vdevs[i].cam = NULL;
	}

	platform_device_unregister(pdata->isp);

	v4l2_device_unregister(&cam->v4l2_dev);
	media_device_unregister(&cam->media_dev);

	kfree(cam);
	return 0;
}

static int omap34xxcam_probe(struct platform_device *pdev)
{
	struct omap34xxcam_platform_data *pdata = pdev->dev.platform_data;
	struct omap34xxcam_device *cam;
	struct isp_device *isp = NULL;
	int ret;
	int i;

	if (pdata == NULL)
		return -EINVAL;

	cam = kzalloc(sizeof(*cam), GFP_KERNEL);
	if (!cam) {
		printk(KERN_ERR "%s: could not allocate memory\n", __func__);
		return -ENOMEM;
	}

	cam->pdata = pdata;
	atomic_set(&cam->refcount, 1);

	cam->media_dev.dev = &pdev->dev;
	ret = media_device_register(&cam->media_dev);
	if (ret < 0) {
		printk(KERN_ERR "%s: Media device registration failed (%d)\n",
			__func__, ret);
		goto err;
	}

	cam->v4l2_dev.mdev = &cam->media_dev;
	ret = v4l2_device_register(&pdev->dev, &cam->v4l2_dev);
	if (ret < 0) {
		printk(KERN_ERR "%s: V4L2 device registration failed (%d)\n",
			__func__, ret);
		goto err;
	}

	/* We need to check rval just once. The place is here. */
	platform_device_register(pdata->isp);

	cam->isp = platform_get_drvdata(pdata->isp);
	isp = isp_get(cam->isp);
	if (isp == NULL) {
		printk(KERN_ERR "%s: can't get ISP\n", __func__);
		ret = -EBUSY;
		goto err;
	}

	for (i = 0; i < OMAP34XXCAM_VIDEODEVS; i++) {
		struct omap34xxcam_videodev *vdev = &cam->vdevs[i];
		struct media_entity *sensor;
		struct media_entity *input;
		unsigned int flags;
		unsigned int pad;

		mutex_init(&vdev->mutex);
		vdev->index = i;
		vdev->cam = cam;

		ret = omap34xxcam_device_register(vdev, pdata->subdevs[i]);
		if (ret < 0)
			goto err;

		/* Connect the sensor to the correct interface module. Parallel
		 * sensors are connected directly to the CCDC, while serial
		 * sensors are connected to the CSI2a, CCP2b or CSI2c receiver
		 * through CSIPHY1 or CSIPHY2.
		 */
		switch (pdata->sensors[i].interface) {
		case ISP_INTERFACE_PARALLEL:
			input = &isp->isp_ccdc.subdev.entity;
			pad = CCDC_PAD_SINK;
			flags = 0;
			break;

		case ISP_INTERFACE_CSI2A_PHY2:
			input = &isp->isp_csi2a.subdev.entity;
			pad = CSI2_PAD_SINK;
			flags = MEDIA_LINK_FLAG_IMMUTABLE
			      | MEDIA_LINK_FLAG_ACTIVE;
			break;

		case ISP_INTERFACE_CCP2B_PHY1:
		case ISP_INTERFACE_CCP2B_PHY2:
			input = &isp->isp_ccp2.subdev.entity;
			pad = CCP2_PAD_SINK;
			flags = 0;
		break;

		case ISP_INTERFACE_CSI2C_PHY1:
			input = &isp->isp_csi2c.subdev.entity;
			pad = CSI2_PAD_SINK;
			flags = MEDIA_LINK_FLAG_IMMUTABLE
			      | MEDIA_LINK_FLAG_ACTIVE;
			break;

		default:
			printk(KERN_ERR "%s: invalid interface type %u for "
				"sensor %u\n", __func__,
				pdata->sensors[i].interface, i);
			ret = -EINVAL;
			goto err;
		}

		sensor = &vdev->vdev_sensor->entity;
		ret = media_entity_create_link(sensor, 0, input, pad, flags);
		if (ret < 0)
			goto err;

		ret = media_entity_create_link(
			&isp->isp_ccdc.subdev.entity, CCDC_PAD_SOURCE_OF,
			&vdev->video.video.entity, 0, 0);
		if (ret < 0)
			goto err;

		ret = media_entity_create_link(
			&isp->isp_res.subdev.entity, RESZ_PAD_SOURCE,
			&vdev->video.video.entity, 0, 0);
		if (ret < 0)
			goto err;

	}

	for (i = 0; i < OMAP34XXCAM_VIDEODEVS; i++) {
		struct omap34xxcam_videodev *vdev = &cam->vdevs[i];

		ret = isp_video_register(&vdev->video, &vdev->cam->v4l2_dev);
		if (ret < 0) {
			printk(KERN_ERR "%s: could not register video device (%d)\n",
				__func__, ret);
			goto err;
		}
	}

	ret = omap3isp_register_entities(pdata->isp, &cam->v4l2_dev);
	if (ret < 0) {
		printk(KERN_ERR "%s: Can't register ISP subdevices (%d)\n",
			__func__, ret);
		goto err;
	}

	isp_put(isp);
	atomic_set(&cam->refcount, 0);
	return 0;

err:
	isp_put(isp);
	omap34xxcam_remove(pdev);
	return ret;
}

#ifdef CONFIG_PM
static int omap34xxcam_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int omap34xxcam_resume(struct platform_device *pdev)
{
	return 0;
}
#else

#define omap34xxcam_suspend	NULL
#define omap34xxcam_resume	NULL

#endif /* CONFIG_PM */

static struct platform_driver omap34xxcam_driver = {
	.probe = omap34xxcam_probe,
	.remove = omap34xxcam_remove,
	.suspend = omap34xxcam_suspend,
	.resume = omap34xxcam_resume,
	.driver = {
		.name = CAM_NAME,
		.owner = THIS_MODULE,
	},
};

/*
 *
 * Module initialisation and deinitialisation
 *
 */

static int __init omap34xxcam_init(void)
{
	return platform_driver_register(&omap34xxcam_driver);
}

static void omap34xxcam_exit(void)
{
	platform_driver_unregister(&omap34xxcam_driver);
}

MODULE_AUTHOR("Sakari Ailus <sakari.ailus@nokia.com>");
MODULE_DESCRIPTION("OMAP34xx Video for Linux camera driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("omap34xxcam-mod");

module_init(omap34xxcam_init);
module_exit(omap34xxcam_exit);

