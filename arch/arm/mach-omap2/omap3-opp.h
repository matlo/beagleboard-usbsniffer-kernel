#ifndef __OMAP3_OPP_H_
#define __OMAP3_OPP_H_

#include <plat/omap-pm.h>

/* MPU speeds */
#define S1000M 1000000000
#define S800M   800000000
#define S720M   720000000
#define S600M   600000000
#define S550M   550000000
#define S500M   500000000
#define S250M   250000000
#define S300M   300000000
#define S125M   125000000

/* DSP speeds */
#define S875M   875000000
#define S660M   660000000
#define S520M   520000000
#define S430M   430000000
#define S400M   400000000
#define S360M   360000000
#define S260M   260000000
#define S180M   180000000
#define S90M    90000000

/* L3 speeds */
#define S83M    83000000
#define S100M   100000000
#define S166M   166000000
#define S200M   200000000

/* VDD OPP identifiers */
#define VDD1_OPP	0x1
#define VDD2_OPP	0x2

/* VDD1 OPPS */
#define VDD1_OPP1	0x1
#define VDD1_OPP2	0x2
#define VDD1_OPP3	0x3
#define VDD1_OPP4	0x4
#define VDD1_OPP5	0x5
#define VDD1_OPP6	0x6

/* VDD2 OPPS */
#define VDD2_OPP1	0x1
#define VDD2_OPP2	0x2
#define VDD2_OPP3	0x3

/**
 * Get minimum OPP for VDD1
 */
static inline u8 get_min_vdd1(void)
{
	return VDD1_OPP1;
}

/**
 * Get maximum OPP for VDD1
 */
static inline u8 get_max_vdd1(void)
{
	if (cpu_is_omap3630()) {
		return VDD1_OPP4;
	} else {
		return VDD1_OPP6;
	}
}

/**
 * Get minimum OPP for VDD2
 */
static inline u8 get_min_vdd2(void)
{
	return VDD2_OPP1;
}

/**
 * Get maximum OPP for VDD2
 */
static inline u8 get_max_vdd2(void)
{
	if (cpu_is_omap3630()) {
		return VDD2_OPP2;
	} else {
		return VDD2_OPP3;
	}
}

extern struct omap_opp omap35x_mpu_rate_table[];
extern struct omap_opp omap35x_l3_rate_table[];
extern struct omap_opp omap35x_dsp_rate_table[];

extern struct omap_opp omap37x_mpu_rate_table[];
extern struct omap_opp omap37x_l3_rate_table[];
extern struct omap_opp omap37x_dsp_rate_table[];

#endif
