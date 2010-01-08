#ifndef __OMAP3_OPP_H_
#define __OMAP3_OPP_H_

#include <plat/omap-pm.h>

/* MPU speeds */
#define S720M   720000000
#define S600M   600000000
#define S550M   550000000
#define S500M   500000000
#define S250M   250000000
#define S125M   125000000

/* DSP speeds */
#define S520M   520000000
#define S430M   430000000
#define S400M   400000000
#define S360M   360000000
#define S180M   180000000
#define S90M    90000000

/* L3 speeds */
#define S83M    83000000
#define S166M   166000000

extern struct omap_opp omap3_mpu_rate_table[];
extern struct omap_opp omap3_l3_rate_table[];
extern struct omap_opp omap3_dsp_rate_table[];

#endif
