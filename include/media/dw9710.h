/*
 * include/media/dw9710.h
 *
 * Public defines for Auto Focus device
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * Contributors:
 *	Sergio Aguirre <saaguirre@ti.com>
 * 	Troy Laramy
 * 	Mohit Jalori
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 */

#ifndef DW9710_H
#define DW9710_H

#define DW9710_NAME 			"dw9710"
#define DW9710_AF_I2C_ADDR		0x0C

/**
 * struct dw9710_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @priv_data_set: device private data (pointer) access function
 */
struct dw9710_platform_data {
	int (*power_set)(enum v4l2_power power);
	int (*priv_data_set)(void *);
};

#endif /* End of of DW9710_H */
