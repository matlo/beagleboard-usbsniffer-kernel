/*
 * OMAP OPP Interface
 *
 * Copyright (C) 2009 Texas Instruments Incorporated.
 *	Nishanth Menon
 * Copyright (C) 2009 Deep Root Systems, LLC.
 *	Kevin Hilman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_ARM_OMAP_OPP_H
#define __ASM_ARM_OMAP_OPP_H

extern struct omap_opp *mpu_opps;
extern struct omap_opp *dsp_opps;
extern struct omap_opp *l3_opps;

/**
 * struct omap_opp - OMAP OPP description structure
 * @enabled:	true/false - marking this OPP as enabled/disabled
 * @rate:	Frequency in hertz
 * @opp_id:	(DEPRECATED) opp identifier
 * @vsel:	Voltage in volt processor level(this usage is
 *		DEPRECATED to use Voltage in microvolts in future)
 *		uV = ((vsel * 12.5) + 600) * 1000
 *
 * This structure stores the OPP information for a given domain.
 * Due to legacy reasons, this structure is currently exposed and
 * will soon be removed elsewhere and will only be used as a handle
 * from the OPP internal referencing mechanism
 */
struct omap_opp {
	bool enabled;
	unsigned long rate;
	u8 __deprecated opp_id;
	u16 __deprecated vsel;
};

/**
 * opp_get_voltage() - Gets the voltage corresponding to an opp
 * @opp:	opp for which voltage has to be returned for
 *
 * Return voltage in micro volt corresponding to the opp, else
 * return 0
 */
unsigned long opp_get_voltage(const struct omap_opp *opp);

/**
 * opp_get_freq() - Gets the frequency corresponding to an opp
 * @opp:	opp for which frequency has to be returned for
 *
 * Return frequency in hertz corresponding to the opp, else
 * return 0
 */
unsigned long opp_get_freq(const struct omap_opp *opp);

/**
 * opp_get_opp_count() - Get number of opps enabled in the opp list
 * @num:	returns the number of opps
 * @oppl:	opp list
 *
 * This functions returns the number of opps if there are any OPPs enabled,
 * else returns corresponding error value.
 */
int opp_get_opp_count(struct omap_opp *oppl);

/**
 * opp_find_freq_exact() - search for an exact frequency
 * @oppl:	OPP list
 * @freq:	frequency to search for
 * @enabled:	enabled/disabled OPP to search for
 *
 * searches for the match in the opp list and returns handle to the matching
 * opp if found, else returns ERR_PTR in case of error and should be handled
 * using IS_ERR.
 *
 * Note enabled is a modifier for the search. if enabled=true, then the match is
 * for exact matching frequency and is enabled. if true, the match is for exact
 * frequency which is disabled.
 */
struct omap_opp *opp_find_freq_exact(struct omap_opp *oppl,
				     unsigned long freq, bool enabled);

/* XXX This documentation needs fixing */

/**
 * opp_find_freq_floor() - Search for an rounded freq
 * @oppl:	Starting list
 * @freq:	Start frequency
 *
 * Search for the lower *enabled* OPP from a starting freq
 * from a start opp list.
 *
 * Returns *opp and *freq is populated with the next match, else
 * returns NULL opp if found, else returns ERR_PTR in case of error.
 *
 * Example usages:
 *	* find match/next lowest available frequency
 *	freq = 350000;
 *	opp = opp_find_freq_floor(oppl, &freq)))
 *	if (IS_ERR(opp))
 *		pr_err ("unable to find a lower frequency\n");
 *	else
 *		pr_info("match freq = %ld\n", freq);
 *
 *	* print all supported frequencies in descending order *
 *	opp = oppl;
 *	freq = ULONG_MAX;
 *	while (!IS_ERR(opp = opp_find_freq_floor(opp, &freq)) {
 *		pr_info("freq = %ld\n", freq);
 *		freq--; * for next lower match *
 *	}
 *
 * NOTE: if we set freq as ULONG_MAX and search low, we get the
 * highest enabled frequency
 */
struct omap_opp *opp_find_freq_floor(struct omap_opp *oppl,
				     unsigned long *freq);

/* XXX This documentation needs fixing */

/**
 * opp_find_freq_ceil() - Search for an rounded freq
 * @oppl:	Starting list
 * @freq:	Start frequency
 *
 * Search for the higher *enabled* OPP from a starting freq
 * from a start opp list.
 *
 * Returns *opp and *freq is populated with the next match, else
 * returns NULL opp if found, else returns ERR_PTR in case of error.
 *
 * Example usages:
 *	* find match/next highest available frequency
 *	freq = 350000;
 *	opp = opp_find_freq_ceil(oppl, &freq))
 *	if (IS_ERR(opp))
 *		pr_err ("unable to find a higher frequency\n");
 *	else
 *		pr_info("match freq = %ld\n", freq);
 *
 *	* print all supported frequencies in ascending order *
 *	opp = oppl;
 *	freq = 0;
 *	while (!IS_ERR(opp = opp_find_freq_ceil(opp, &freq)) {
 *		pr_info("freq = %ld\n", freq);
 *		freq++; * for next higher match *
 *	}
 */
struct omap_opp *opp_find_freq_ceil(struct omap_opp *oppl, unsigned long *freq);


/**
 * struct omap_opp_def - OMAP OPP Definition
 * @enabled:	True/false - is this OPP enabled/disabled by default
 * @freq:	Frequency in hertz corresponding to this OPP
 * @u_volt:	Nominal voltage in microvolts corresponding to this OPP
 *
 * OMAP SOCs have a standard set of tuples consisting of frequency and voltage
 * pairs that the device will support per voltage domain. This is called
 * Operating Points or OPP. The actual definitions of OMAP Operating Points
 * varies over silicon within the same family of devices. For a specific
 * domain, you can have a set of {frequency, voltage} pairs and this is denoted
 * by an array of omap_opp_def. As the kernel boots and more information is
 * available, a set of these are activated based on the precise nature of
 * device the kernel boots up on. It is interesting to remember that each IP
 * which belongs to a voltage domain may define their own set of OPPs on top
 * of this - but this is handled by the appropriate driver.
 */
struct omap_opp_def {
	bool enabled;
	unsigned long freq;
	unsigned long u_volt;
};

/*
 * Initialization wrapper used to define an OPP
 * to point at the end of a terminator of a list of OPPs,
 * use OMAP_OPP_DEF(0, 0, 0)
 */
#define OMAP_OPP_DEF(_enabled, _freq, _uv)	\
{						\
	.enabled	= _enabled,		\
	.freq		= _freq,		\
	.u_volt		= _uv,			\
}

/**
 * opp_init_list() - Initialize an opp list from the opp definitions
 * @opp_defs:	Initial opp definitions to create the list.
 *
 * This function creates a list of opp definitions and returns a handle.
 * This list can be used to further validation/search/modifications. New
 * opp entries can be added to this list by using opp_add().
 *
 * In the case of error, ERR_PTR is returned to the caller and should be
 * appropriately handled with IS_ERR.
 */
struct omap_opp __init *opp_init_list(const struct omap_opp_def *opp_defs);

/**
 * opp_add()  - Add an OPP table from a table definitions
 * @oppl:	List to add the OPP to
 * @opp_def:	omap_opp_def to describe the OPP which we want to add to list.
 *
 * This function adds an opp definition to the opp list and returns
 * a handle representing the new OPP list. This handle is then used for further
 * validation, search, modification operations on the OPP list.
 *
 * This function returns the pointer to the allocated list through oppl if
 * success, else corresponding ERR_PTR value. Caller should NOT free the oppl.
 * opps_defs can be freed after use.
 *
 * NOTE: caller should assume that on success, oppl is probably populated with
 * a new handle and the new handle should be used for further referencing
 */
struct omap_opp *opp_add(struct omap_opp *oppl,
			 const struct omap_opp_def *opp_def);

/**
 * opp_enable() - Enable a specific OPP
 * @opp:	Pointer to opp
 *
 * Enables a provided opp. If the operation is valid, this returns 0, else the
 * corresponding error value.
 *
 * OPP used here is from the the opp_is_valid/opp_has_freq or other search
 * functions
 */
int opp_enable(struct omap_opp *opp);

/**
 * opp_disable() - Disable a specific OPP
 * @opp:	Pointer to opp
 *
 * Disables a provided opp. If the operation is valid, this returns 0, else the
 * corresponding error value.
 *
 * OPP used here is from the the opp_is_valid/opp_has_freq or other search
 * functions
 */
int opp_disable(struct omap_opp *opp);

#endif		/* __ASM_ARM_OMAP_OPP_H */
