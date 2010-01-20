/*
 * arch/arm/mach-omap2/cpufreq34xx.c
 * OMAP3 resource init/change_level/validate_level functions
 *
 * Copyright (C) 2009 - 2010 Texas Instruments Incorporated.
 *	Nishanth Menon
 * Copyright (C) 2009 - 2010 Deep Root Systems, LLC.
 *	Kevin Hilman
 * Copyright (C) 2010 Nokia Corporation.
 *      Eduardo Valentin
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 * History:
 *
 */

#include <linux/module.h>
#include <linux/err.h>

#include <plat/opp.h>
#include <plat/cpu.h>

static struct omap_opp_def __initdata omap34xx_mpu_rate_table[] = {
	/* OPP1 */
	OMAP_OPP_DEF(true, 125000000, 975000),
	/* OPP2 */
	OMAP_OPP_DEF(true, 250000000, 1075000),
	/* OPP3 */
	OMAP_OPP_DEF(true, 500000000, 1200000),
	/* OPP4 */
	OMAP_OPP_DEF(true, 550000000, 1270000),
	/* OPP5 */
	OMAP_OPP_DEF(true, 600000000, 1350000),
	/* Terminator */
	OMAP_OPP_DEF(0, 0, 0)
};

static struct omap_opp_def __initdata omap34xx_l3_rate_table[] = {
	/* OPP1 */
	OMAP_OPP_DEF(false, 0, 975000),
	/* OPP2 */
	OMAP_OPP_DEF(true, 83000000, 1050000),
	/* OPP3 */
	OMAP_OPP_DEF(true, 166000000, 1150000),
	/* Terminator */
	OMAP_OPP_DEF(0, 0, 0)
};

static struct omap_opp_def __initdata omap34xx_dsp_rate_table[] = {
	/* OPP1 */
	OMAP_OPP_DEF(true, 90000000, 975000),
	/* OPP2 */
	OMAP_OPP_DEF(true, 180000000, 1075000),
	/* OPP3 */
	OMAP_OPP_DEF(true, 360000000, 1200000),
	/* OPP4 */
	OMAP_OPP_DEF(true, 400000000, 1270000),
	/* OPP5 */
	OMAP_OPP_DEF(true, 430000000, 1350000),
	/* Terminator */
	OMAP_OPP_DEF(0, 0, 0)
};

static struct omap_opp_def __initdata omap36xx_mpu_rate_table[] = {
	/* OPP1 - OPP50 */
	OMAP_OPP_DEF(true,  300000000, 930000),
	/* OPP2 - OPP100 */
	OMAP_OPP_DEF(true,  600000000, 1100000),
	/* OPP3 - OPP-Turbo */
	OMAP_OPP_DEF(false, 800000000, 1260000),
	/* OPP4 - OPP-SB */
	OMAP_OPP_DEF(false, 1000000000, 1310000),
	/* Terminator */
	OMAP_OPP_DEF(0, 0, 0)
};

static struct omap_opp_def __initdata omap36xx_l3_rate_table[] = {
	/* OPP1 - OPP50 */
	OMAP_OPP_DEF(true, 100000000, 930000),
	/* OPP2 - OPP100, OPP-Turbo, OPP-SB */
	OMAP_OPP_DEF(true, 200000000, 1137500),
	/* Terminator */
	OMAP_OPP_DEF(0, 0, 0)
};

static struct omap_opp_def __initdata omap36xx_dsp_rate_table[] = {
	/* OPP1 - OPP50 */
	OMAP_OPP_DEF(true,  260000000, 930000),
	/* OPP2 - OPP100 */
	OMAP_OPP_DEF(true,  520000000, 1100000),
	/* OPP3 - OPP-Turbo */
	OMAP_OPP_DEF(false, 660000000, 1260000),
	/* OPP4 - OPP-SB */
	OMAP_OPP_DEF(false, 875000000, 1310000),
	/* Terminator */
	OMAP_OPP_DEF(0, 0, 0)
};

void __init omap3_pm_init_opp_table(void)
{
	int i;
	struct omap_opp_def **omap3_opp_def_list;
	struct omap_opp_def *omap34xx_opp_def_list[] = {
		omap34xx_mpu_rate_table,
		omap34xx_l3_rate_table,
		omap34xx_dsp_rate_table
	};
	struct omap_opp_def *omap36xx_opp_def_list[] = {
		omap36xx_mpu_rate_table,
		omap36xx_l3_rate_table,
		omap36xx_dsp_rate_table
	};
	struct omap_opp **omap3_rate_tables[] = {
		&mpu_opps,
		&l3_opps,
		&dsp_opps
	};

	omap3_opp_def_list = cpu_is_omap3630() ? omap36xx_opp_def_list :
				omap34xx_opp_def_list;
	for (i = 0; i < ARRAY_SIZE(omap3_rate_tables); i++) {
		*omap3_rate_tables[i] = opp_init_list(omap3_opp_def_list[i]);
		/* We dont want half configured system at the moment */
		BUG_ON(IS_ERR(omap3_rate_tables[i]));
	}
}

