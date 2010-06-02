/*
 * V4L2 subdev userspace API
 *
 * Copyright (C) 2010 Nokia
 *
 * Contributors:
 *	Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef __LINUX_V4L2_SUBDEV_H
#define __LINUX_V4L2_SUBDEV_H

#include <linux/ioctl.h>
#include <linux/v4l2-mediabus.h>

enum v4l2_subdev_format {
	V4L2_SUBDEV_FORMAT_PROBE = 0,
	V4L2_SUBDEV_FORMAT_ACTIVE = 1,
};

/**
 * struct v4l2_subdev_pad_format
 */
struct v4l2_subdev_pad_format {
	__u32 which;
	__u32 pad;
	struct v4l2_mbus_framefmt format;
};

/**
 * struct v4l2_subdev_pad_frame_rate
 */
struct v4l2_subdev_frame_interval {
	struct v4l2_fract interval;
	__u32 reserved[6];
};

/**
 * struct v4l2_subdev_pad_mbus_code_enum
 */
struct v4l2_subdev_pad_mbus_code_enum {
	__u32 pad;
	__u32 index;
	__u32 code;
	__u32 reserved[5];
};

#define VIDIOC_SUBDEV_G_FMT	_IOWR('V',  4, struct v4l2_subdev_pad_format)
#define VIDIOC_SUBDEV_S_FMT	_IOWR('V',  5, struct v4l2_subdev_pad_format)
#define VIDIOC_SUBDEV_G_FRAME_INTERVAL \
				_IOWR('V', 6, struct v4l2_subdev_frame_interval)
#define VIDIOC_SUBDEV_S_FRAME_INTERVAL \
				_IOWR('V', 7, struct v4l2_subdev_frame_interval)
#define VIDIOC_SUBDEV_ENUM_MBUS_CODE \
			_IOWR('V', 8, struct v4l2_subdev_pad_mbus_code_enum)

#endif

