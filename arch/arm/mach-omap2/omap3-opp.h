#ifndef __OMAP3_OPP_H_
#define __OMAP3_OPP_H_

#include <plat/omap-pm.h>

/* MPU speeds */
#define S600M   600000000
#define S550M   550000000
#define S500M   500000000
#define S250M   250000000
#define S125M   125000000

/* DSP speeds */
#define S430M   430000000
#define S400M   400000000
#define S360M   360000000
#define S180M   180000000
#define S90M    90000000

/* L3 speeds */
#define S83M    83000000
#define S166M   166000000

static struct omap_opp omap3_mpu_rate_table[] = {
	{0, 0, 0, 0},
	/*OPP1*/
	{true, S125M, VDD1_OPP1, 0x1E},
	/*OPP2*/
	{true, S250M, VDD1_OPP2, 0x26},
	/*OPP3*/
	{true, S500M, VDD1_OPP3, 0x30},
	/*OPP4*/
	{true, S550M, VDD1_OPP4, 0x36},
	/*OPP5*/
	{true, S600M, VDD1_OPP5, 0x3C},
};

static struct omap_opp omap3_l3_rate_table[] = {
	{0, 0, 0, 0},
	/*OPP1*/
	{false, 0, VDD2_OPP1, 0x1E},
	/*OPP2*/
	{true, S83M, VDD2_OPP2, 0x24},
	/*OPP3*/
	{true, S166M, VDD2_OPP3, 0x2C},
};

static struct omap_opp omap3_dsp_rate_table[] = {
	{0, 0, 0, 0},
	/*OPP1*/
	{true, S90M, VDD1_OPP1, 0x1E},
	/*OPP2*/
	{true, S180M, VDD1_OPP2, 0x26},
	/*OPP3*/
	{true, S360M, VDD1_OPP3, 0x30},
	/*OPP4*/
	{true, S400M, VDD1_OPP4, 0x36},
	/*OPP5*/
	{true, S430M, VDD1_OPP5, 0x3C},
};

#endif
