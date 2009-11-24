/*
 * OMAP OPP Interface
 *
 * Copyright (C) 2009 Texas Instruments Incorporated.
 *	Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/slab.h>

#include <plat/opp.h>

/*
 * DEPRECATED: Meant to detect end of opp array
 * This is meant to help co-exist with current SRF etc
 * TODO: REMOVE!
 */
#define OPP_TERM(opp) (!(opp)->rate && !(opp)->vsel && !(opp)->enabled)

/*
 * DEPRECATED: Meant to convert vsel value to uVolt
 * This is meant to help co-exist with current SRF etc
 * TODO: REMOVE!
 */
static inline unsigned long vsel_to_uv(const u8 vsel)
{
	return (((vsel * 125) + 6000)) * 100;
}

/*
 * DEPRECATED: Meant to convert uVolt to vsel value
 * This is meant to help co-exist with current SRF etc
 * TODO: REMOVE!
 */
static inline unsigned char uv_to_vsel(unsigned long uV)
{
	return ((uV / 100) - 6000) / 125;
}

unsigned long opp_get_voltage(const struct omap_opp *opp)
{
	if (unlikely(!opp || IS_ERR(opp)) || !opp->enabled) {
		pr_err("%s: Invalid parameters being passed\n", __func__);
		return 0;
	}
	return vsel_to_uv(opp->vsel);
}

unsigned long opp_get_freq(const struct omap_opp *opp)
{
	if (unlikely(!opp || IS_ERR(opp)) || !opp->enabled) {
		pr_err("%s: Invalid parameters being passed\n", __func__);
		return 0;
	}
	return opp->rate;
}

int opp_get_opp_count(const struct omap_opp *oppl)
{
	struct omap_opp *opp;
	u8 n = 0;

	if (unlikely(!oppl || IS_ERR(oppl))) {
		pr_err("%s: Invalid parameters being passed\n", __func__);
		return -EINVAL;
	}
	opp = (struct omap_opp *)oppl;
	opp++;			/* skip initial terminator */
	while (!OPP_TERM(opp)) {
		if (opp->enabled)
			n++;
		opp++;
	}
	return n;
}

struct omap_opp *opp_find_freq_exact(struct omap_opp *oppl,
				     unsigned long freq, bool enabled)
{
	struct omap_opp *opp = (struct omap_opp *)oppl;

	if (unlikely(!oppl || IS_ERR(oppl))) {
		pr_err("%s: Invalid parameters being passed\n", __func__);
		return ERR_PTR(-EINVAL);
	}
	/* skip initial terminator */
	if (OPP_TERM(opp))
		opp++;
	while (!OPP_TERM(opp)) {
		if ((opp->rate == freq) && (opp->enabled == enabled))
			break;
		opp++;
	}

	return OPP_TERM(opp) ? ERR_PTR(-ENOENT) : opp;
}

struct omap_opp *opp_find_freq_approx(struct omap_opp *oppl,
				      unsigned long *freq, u8 dir_flag)
{
	struct omap_opp *opp = (struct omap_opp *)oppl;

	if (unlikely(!oppl || IS_ERR(oppl) || !freq || IS_ERR(freq))) {
		pr_err("%s: Invalid parameters being passed\n", __func__);
		return ERR_PTR(-EINVAL);
	}
	/* skip initial terminator */
	if (OPP_TERM(opp)) {
		opp++;
		/* If searching init list for a high val, skip to very top */
		if (dir_flag == OPP_SEARCH_LOW)
			while (!OPP_TERM(opp + 1))
				opp++;
	}
	while (!OPP_TERM(opp)) {
		if (opp->enabled &&
		    (((dir_flag == OPP_SEARCH_HIGH) && (opp->rate >= *freq)) ||
		     ((dir_flag == OPP_SEARCH_LOW) && (opp->rate <= *freq))))
			break;
		opp += (dir_flag == OPP_SEARCH_LOW) ? -1 : 1;
	}

	if (OPP_TERM(opp))
		return ERR_PTR(-ENOENT);

	*freq = opp->rate;
	return opp;
}

/* wrapper to reuse converting opp_def to opp struct */
static void omap_opp_populate(struct omap_opp *opp,
			      const struct omap_opp_def *opp_def)
{
	opp->rate = opp_def->freq;
	opp->enabled = opp_def->enabled;
	opp->vsel = uv_to_vsel(opp_def->u_volt);
	/* round off to higher voltage */
	if (opp_def->u_volt > vsel_to_uv(opp->vsel))
		opp->vsel++;
}

struct omap_opp *opp_add(struct omap_opp *oppl,
			 const struct omap_opp_def *opp_def)
{
	struct omap_opp *opp, *oppt, *oppr;
	u8 n, i, ins;

	if (unlikely(!oppl || IS_ERR(oppl) || !opp_def || IS_ERR(opp_def))) {
		pr_err("%s: Invalid params being passed\n", __func__);
		return ERR_PTR(-EINVAL);
	}
	/* need a start terminator.. */
	if (unlikely(!OPP_TERM(oppl))) {
		pr_err("%s: Expected a start terminator!!\n", __func__);
		return ERR_PTR(-EINVAL);
	}
	n = 0;
	opp = oppl;
	opp++;
	while (!OPP_TERM(opp)) {
		n++;
		opp++;
	}
	/* lets now reallocate memory */
	oppr = kmalloc(sizeof(struct omap_opp) * (n + 3), GFP_KERNEL);
	if (!oppr) {
		pr_err("%s: No memory for new opp array\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	/* Simple insertion sort */
	opp = oppl;
	oppt = oppr;
	ins = 0;
	i = 0;
	do {
		if (ins || opp->rate < opp_def->freq) {
			memcpy(oppt, opp, sizeof(struct omap_opp));
			opp++;
		} else {
			omap_opp_populate(oppt, opp_def);
			ins++;
		}
		oppt->opp_id = i;
		oppt++;
		i++;
	} while (!OPP_TERM(opp));

	/* If nothing got inserted, this belongs to the end */
	if (!ins) {
		omap_opp_populate(oppt, opp_def);
		oppt->opp_id = i;
		oppt++;
	}
	/* Put the terminator back on */
	memcpy(oppt, opp, sizeof(struct omap_opp));

	/* Free the old list */
	kfree(oppl);

	return oppr;
}

struct omap_opp __init *opp_init_list(const struct omap_opp_def *opp_defs)
{
	struct omap_opp_def *t = (struct omap_opp_def *)opp_defs;
	struct omap_opp *opp, *oppl;
	u8 n = 0, i = 1;

	if (unlikely(!opp_defs || IS_ERR(opp_defs))) {
		pr_err("%s: Invalid params being passed\n", __func__);
		return ERR_PTR(-EINVAL);
	}
	/* Grab a count */
	while (t->enabled || t->freq || t->u_volt) {
		n++;
		t++;
	}

	oppl = kmalloc(sizeof(struct omap_opp) * (n + 2), GFP_KERNEL);
	if (!oppl) {
		pr_err("%s: No memory for opp array\n", __func__);
		return ERR_PTR(-ENOMEM);
	}
	opp = oppl;
	/* Setup start terminator - SRF depends on this for indexing :( */
	opp->rate = 0;
	opp->enabled = 0;
	opp->vsel = 0;
	opp++;
	while (n) {
		omap_opp_populate(opp, opp_defs);
		opp->opp_id = i;
		n--;
		opp++;
		opp_defs++;
		i++;
	}
	/* Setup terminator - this is for our search algos */
	opp->rate = 0;
	opp->enabled = 0;
	opp->vsel = 0;
	return oppl;
}

int opp_enable(struct omap_opp *opp)
{
	if (unlikely(!opp || IS_ERR(opp))) {
		pr_err("%s: Invalid parameters being passed\n", __func__);
		return -EINVAL;
	}
	opp->enabled = true;
	return 0;
}

int opp_disable(struct omap_opp *opp)
{
	if (unlikely(!opp || IS_ERR(opp))) {
		pr_err("%s: Invalid parameters being passed\n", __func__);
		return -EINVAL;
	}
	opp->enabled = false;
	return 0;
}
