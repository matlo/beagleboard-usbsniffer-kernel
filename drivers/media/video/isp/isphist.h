/*
 * isphist.h
 *
 * Header file for HISTOGRAM module in TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contributors:
 * 	David Cohen <david.cohen@nokia.com>
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

#ifndef OMAP_ISP_HIST_H
#define OMAP_ISP_HIST_H

#include <plat/isp_user.h>

#define ISPHIST_IN_BIT_WIDTH_CCDC	10

struct isp_device;

int isphist_init(struct isp_device *isp);
void isphist_cleanup(struct isp_device *isp);

#endif /* OMAP_ISP_HIST */
