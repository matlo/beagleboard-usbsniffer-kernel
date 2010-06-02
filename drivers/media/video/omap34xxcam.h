/*
 * omap34xxcam.h
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

#ifndef OMAP34XXCAM_H
#define OMAP34XXCAM_H

#include <asm/atomic.h>
#include <media/media-device.h>
#include <media/v4l2-device.h>
#include "isp/isp.h"

#define CAM_NAME			"omap34xxcam"
#define CAM_SHORT_NAME			"omap3"

#define OMAP34XXCAM_XCLK_NONE	-1
#define OMAP34XXCAM_XCLK_A	0
#define OMAP34XXCAM_XCLK_B	1

#define OMAP34XXCAM_SUBDEV_SENSOR	0
#define OMAP34XXCAM_SUBDEV_LENS		1
#define OMAP34XXCAM_SUBDEV_FLASH	2 /* This is the last subdev! */

#define OMAP34XXCAM_VIDEODEVS		1

struct omap34xxcam_device;
struct omap34xxcam_videodev;
struct isp_video;

/**
 * struct omap34xxcam_sensor_config - struct for omap34xxcam sensor-related
 * 	platform data
 * @capture_mem: Size limit to mmap buffers.
 * @ival_default: Default frame interval for sensor.
 */
struct omap34xxcam_sensor_config {
	enum isp_interface_type interface;
	u32 capture_mem;
	struct v4l2_fract ival_default;
};

struct omap34xxcam_platform_data {
	struct platform_device *isp;
	struct v4l2_subdev_i2c_board_info *subdevs[OMAP34XXCAM_VIDEODEVS];
	struct omap34xxcam_sensor_config sensors[OMAP34XXCAM_VIDEODEVS];
};

/**
 * struct omap34xxcam_videodev - per /dev/video* structure
 * @mutex: serialises access to this structure
 * @cam: pointer to cam hw structure
 * @v4l2_dev: the v4l2 device
 * @sensor: sensor device
 * @subdevs: how many subdevs we have at the moment
 * @vfd: our video device
 * @index: index of this structure in cam->vdevs
 * @users: how many users we have
 * @poweroff_timer: Timer for dispatching poweroff_work
 * @poweroff_work: Work for subdev power state change
 * @streaming: streaming file handle, if streaming is enabled
 * @want_timeperframe: Desired timeperframe
 * @want_pix: Desired pix
 */
struct omap34xxcam_videodev {
	struct mutex mutex; /* serialises access to this structure */
	char name[32];

	struct omap34xxcam_device *cam;

#define vdev_sensor subdev[OMAP34XXCAM_SUBDEV_SENSOR]
	struct v4l2_subdev *subdev[OMAP34XXCAM_SUBDEV_FLASH + 1];

	/* number of subdevs attached */
	int subdevs;

	/*** video device parameters ***/
	struct isp_video video;

	/*** general driver state information ***/
	int index;

	/*** capture data ***/
	struct file *streaming;
	struct v4l2_fract want_timeperframe;
	struct v4l2_pix_format want_pix;
};

#define to_omap34xxcam_videodev(video) \
	container_of(video, struct omap34xxcam_videodev, video)

/**
 * struct omap34xxcam_device - per-device data structure
 * @vdevs: /dev/video specific structures
 */
struct omap34xxcam_device {
	struct omap34xxcam_videodev vdevs[OMAP34XXCAM_VIDEODEVS];
	struct omap34xxcam_platform_data *pdata;
	struct v4l2_device v4l2_dev;
	struct media_device media_dev;
	struct isp_device *isp;
	atomic_t refcount;
};

#define to_omap34xxcam_device(dev) \
	container_of(dev, struct omap34xxcam_device, v4l2_dev)

#endif /* ifndef OMAP34XXCAM_H */
