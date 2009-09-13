/*
 * tps61059.h - Register definitions for the TPS61059 Flash Chip.
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * Contributors:
 * 	Pallavi Kulkarni <p-kulkarni@ti.com>
 * 	Sergio Aguirre <saaguirre@ti.com>
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#ifndef TPS61059_H
#define TPS61059_H

#include <media/v4l2-int-device.h>

/**
 * struct tps61059_platform_data - platform data values and access functions
 * @power_set: Power state access function, zero is off, non-zero is on.
 * @flash_on: Turn on the flash.
 * @flash_off: Turn off the flash.
 * @update_hw: Depending on the torch intensity, turn on/off torch.
 * @priv_data_set: device private data (pointer) access function
 */
struct tps61059_platform_data {
	void (*flash_on)(void);
	void (*flash_off)(void);
	void (*s_torch_intensity)(u32 value);
	int (*priv_data_set)(void *);
};

#endif /* ifndef TPS61059_H */
