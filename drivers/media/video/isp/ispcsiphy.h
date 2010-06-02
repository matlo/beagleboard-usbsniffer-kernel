/*
 * ispcsiphy.h
 *
 * Copyright (C) 2009 Texas Instruments.
 * Copyright (C) 2010 Nokia Corporation.
 *
 * Contributors:
 * 	Sergio Aguirre <saaguirre@ti.com>
 * 	Dominic Curran <dcurran@ti.com>
 * 	Antti Koskipaa <antti.koskipaa@nokia.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_ISP_CSI_PHY_H
#define OMAP_ISP_CSI_PHY_H

struct isp_csi2_device;

struct csiphy_lane {
	u8 pos;
	u8 pol;
};

#define ISP_CSIPHY2_NUM_DATA_LANES	2
#define ISP_CSIPHY1_NUM_DATA_LANES	1

struct isp_csiphy_lanes_cfg {
	struct csiphy_lane data[ISP_CSIPHY2_NUM_DATA_LANES];
	struct csiphy_lane clk;
};

struct isp_csiphy_dphy_cfg {
	u8 ths_term;
	u8 ths_settle;
	u8 tclk_term;
	unsigned tclk_miss:1;
	u8 tclk_settle;
};

struct isp_csiphy {
	struct isp_device *isp;
	struct mutex mutex;	/* serialize csiphy configuration */
	u8 phy_in_use;
	struct isp_csi2_device *csi2;

	/* mem resources - enums as defined in enum isp_mem_resources */
	unsigned int cfg_regs;
	unsigned int phy_regs;

	u8 num_data_lanes;	/* number of CSI2 Data Lanes supported */
	struct isp_csiphy_lanes_cfg lanes;
	struct isp_csiphy_dphy_cfg dphy;
};

int isp_csiphy_config(struct isp_csiphy *phy, struct isp_csiphy_dphy_cfg *dphy,
		      struct isp_csiphy_lanes_cfg *lanes);

int isp_csiphy_acquire(struct isp_csiphy *phy);
void isp_csiphy_release(struct isp_csiphy *phy);
int isp_csiphy_init(struct isp_device *isp);

#endif	/* OMAP_ISP_CSI_PHY_H */

