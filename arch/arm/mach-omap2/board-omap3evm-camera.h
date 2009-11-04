/*
 * arch/arm/mach-omap2/board-omap3evm-dc.h
 *
 * Copyright (C) 2008 Texas Instruments Inc
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * Contributors:
 *    Anuj Aggarwal <anuj.aggarwal@ti.com>
 *    Sivaraj R <sivaraj@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef __BOARD_OMAP3EVM_DC_H_
#define __BOARD_OMAP3EVM_DC_H_

/* mux id to enable/disable signal routing to different peripherals */
enum omap3evmdc_mux {
	MUX_TVP5146 = 0,
	MUX_CAMERA_SENSOR,
	MUX_EXP_CAMERA_SENSOR,
	NUM_MUX
};

/* enum to enable or disable mux */
enum config_mux {
	DISABLE_MUX,
	ENABLE_MUX
};

#endif		/* __BOARD_OMAP3EVM_DC_H_ */
