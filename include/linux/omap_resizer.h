/*
 * omap_resizer.h
 *
 * Include file for Resizer module wrapper in TI's OMAP3430 ISP
 *
 * Copyright (C) 2010 Texas Instruments, Inc.
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_RESIZER_H
#define OMAP_RESIZER_H

#include <linux/types.h>

/* ioctls definition */
#define RSZ_IOC_BASE		'R'
#define RSZ_IOC_MAXNR		5

/*Ioctl options which are to be passed while calling the ioctl*/
#define RSZ_G_STATUS		_IOWR(RSZ_IOC_BASE, 1, struct rsz_status)
#define RSZ_S_EXP		_IOWR(RSZ_IOC_BASE, 2, __s32)
#define RSZ_S_COEFF		_IOWR(RSZ_IOC_BASE, 3, struct rsz_coeff)
#define RSZ_S_STRT_PHASE	_IOWR(RSZ_IOC_BASE, 4, struct rsz_strt_phase)
#define RSZ_S_LUMA_ENHANCEMENT	_IOWR(RSZ_IOC_BASE, 5, struct rsz_yenh)

enum config_done {
	STATE_CONFIGURED,			/* Resizer driver configured
						 * by application.
						 */
	STATE_NOT_CONFIGURED			/* Resizer driver not
						 * configured by application.
						 */
};

/* Structure Definitions */
struct rsz_strt_phase {
	__u8 horz_strt_phase;
	__u8 vert_strt_phase;
};
struct rsz_coeff {
	__u16 tap4filt_coeffs[32];		/* horizontal filter
						 * coefficients.
						 */
	__u16 tap7filt_coeffs[32];		/* vertical filter
						 * coefficients.
						 */
};
/* used to luma enhancement options */

struct rsz_yenh {
	__s32 type;				/* represents luma enable or
						 * disable.
						 */
	__u8 gain;			/* represents gain. */
	__u8 slop;			/* represents slop. */
	__u8 core;			/* Represents core value. */
};

/* Conatins all the parameters for resizing. This structure
 * is used to configure resiser parameters
 */
struct rsz_params {
	__s32 vert_starting_pixel;		/* for specifying vertical
						 * starting pixel in input.
						 */
	__s32 horz_starting_pixel;		/* for specyfing horizontal
						 * starting pixel in input.
						 */
	__s32 cbilin;				/* # defined, filter with luma
						 * or bi-linear interpolation.
						 */
	__s32 hstph;				/* for specifying horizontal
						 * starting phase.
						 */
	__s32 vstph;				/* for specifying vertical
						 * starting phase.
						 */
	__u16 tap4filt_coeffs[32];		/* horizontal filter
						 * coefficients.
						 */
	__u16 tap7filt_coeffs[32];		/* vertical filter
						 * coefficients.
						 */
	struct rsz_yenh yenh_params;
};

/* Contains the status of hardware and channel */
struct rsz_status {
	__s32 chan_busy;				/* 1: channel is busy,
						 * 0: channel is not busy
						 */
	__s32 hw_busy;				/* 1: hardware is busy,
						 * 0: hardware is not busy
						 */
	__s32 src;				/* # defined, can be either
						 * SD-RAM or CCDC/PREVIEWER
						 */
};

#endif
