/*
 * ov3640.h - Shared settings for the OV3640 CameraChip.
 *
 * Contributors:
 *   Pallavi Kulkarni <p-kulkarni@ti.com>
 *   Sergio Aguirre <saaguirre@ti.com>
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef OV3640_H
#define OV3640_H

#include <media/v4l2-int-device.h>

#define OV3640_I2C_ADDR		(0x78 >> 1)

struct ov3640_platform_data {
	/* Set power state, zero is off, non-zero is on. */
	int (*power_set)(struct v4l2_int_device *s, enum v4l2_power power);
	u32 (*set_xclk)(struct v4l2_int_device *s, u32 xclkfreq);
	int (*priv_data_set)(struct v4l2_int_device *s, void *);
};

#endif /* ifndef OV3640_H */
