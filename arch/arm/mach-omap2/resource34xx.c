/*
 * linux/arch/arm/mach-omap2/resource34xx.c
 * OMAP3 resource init/change_level/validate_level functions
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Rajendra Nayak <rnayak@ti.com>
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

#include <linux/pm_qos_params.h>
#include <linux/cpufreq.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/err.h>

#include <plat/powerdomain.h>
#include <plat/clockdomain.h>
#include <plat/omap34xx.h>
#include <plat/opp_twl_tps.h>

#include "smartreflex.h"
#include "resource34xx.h"
#include "pm.h"
#include "cm.h"
#include "cm-regbits-34xx.h"

#ifndef CONFIG_CPU_IDLE
#warning MPU latency constraints require CONFIG_CPU_IDLE to function!
#endif

/**
 * init_latency - Initializes the mpu/core latency resource.
 * @resp: Latency resource to be initalized
 *
 * No return value.
 */
void init_latency(struct shared_resource *resp)
{
	resp->no_of_users = 0;
	resp->curr_level = RES_LATENCY_DEFAULTLEVEL;
	*((u8 *)resp->resource_data) = 0;
	return;
}

/**
 * set_latency - Adds/Updates and removes the CPU_DMA_LATENCY in *pm_qos_params.
 * @resp: resource pointer
 * @latency: target latency to be set
 *
 * Returns 0 on success, or error values as returned by
 * pm_qos_update_requirement/pm_qos_add_requirement.
 */
int set_latency(struct shared_resource *resp, u32 latency)
{
	u8 *pm_qos_req_added;

	if (resp->curr_level == latency)
		return 0;
	else
		/* Update the resources current level */
		resp->curr_level = latency;

	pm_qos_req_added = resp->resource_data;
	if (latency == RES_LATENCY_DEFAULTLEVEL)
		/* No more users left, remove the pm_qos_req if present */
		if (*pm_qos_req_added) {
			pm_qos_remove_requirement(PM_QOS_CPU_DMA_LATENCY,
							resp->name);
			*pm_qos_req_added = 0;
			return 0;
		}

	if (*pm_qos_req_added) {
		return pm_qos_update_requirement(PM_QOS_CPU_DMA_LATENCY,
						resp->name, latency);
	} else {
		*pm_qos_req_added = 1;
		return pm_qos_add_requirement(PM_QOS_CPU_DMA_LATENCY,
						resp->name, latency);
	}
}

/**
 * init_pd_latency - Initializes the power domain latency resource.
 * @resp: Power Domain Latency resource to be initialized.
 *
 * No return value.
 */
void init_pd_latency(struct shared_resource *resp)
{
	struct pd_latency_db *pd_lat_db;

	resp->no_of_users = 0;
	if (enable_off_mode)
		resp->curr_level = PD_LATENCY_OFF;
	else
		resp->curr_level = PD_LATENCY_RET;
	pd_lat_db = resp->resource_data;
	/* Populate the power domain associated with the latency resource */
	pd_lat_db->pd = pwrdm_lookup(pd_lat_db->pwrdm_name);
	set_pwrdm_state(pd_lat_db->pd, resp->curr_level);
	return;
}

/**
 * set_pd_latency - Updates the curr_level of the power domain resource.
 * @resp: Power domain latency resource.
 * @latency: New latency value acceptable.
 *
 * This function maps the latency in microsecs to the acceptable
 * Power domain state using the latency DB.
 * It then programs the power domain to enter the target state.
 * Always returns 0.
 */
int set_pd_latency(struct shared_resource *resp, u32 latency)
{
	u32 pd_lat_level, ind;
	struct pd_latency_db *pd_lat_db;
	struct powerdomain *pwrdm;

	pd_lat_db = resp->resource_data;
	pwrdm = pd_lat_db->pd;
	pd_lat_level = PD_LATENCY_OFF;
	/* using the latency db map to the appropriate PD state */
	for (ind = 0; ind < PD_LATENCY_MAXLEVEL; ind++) {
		if (pd_lat_db->latency[ind] < latency) {
			pd_lat_level = ind;
			break;
		}
	}

	if (!enable_off_mode && pd_lat_level == PD_LATENCY_OFF)
		pd_lat_level = PD_LATENCY_RET;

	resp->curr_level = pd_lat_level;
	set_pwrdm_state(pwrdm, pd_lat_level);
	return 0;
}

static struct shared_resource *vdd1_resp;
static struct shared_resource *vdd2_resp;
static struct device dummy_mpu_dev;
static struct device dummy_dsp_dev;
static struct device vdd2_dev;
static int vdd1_lock;
static int vdd2_lock;
static struct clk *dpll1_clk, *dpll2_clk, *dpll3_clk;
static int curr_vdd1_opp;
static int curr_vdd2_opp;
static DEFINE_MUTEX(dvfs_mutex);

/**
 * opp_to_freq - convert OPPID to frequency (DEPRECATED)
 * @freq: return frequency back to caller
 * @opp_type: OPP type where we need to look.
 * @opp_id: OPP ID we are searching for
 *
 * return 0 and freq is populated if we find the opp_id, else,
 * we return error
 *
 * NOTE: this function is a standin for the timebeing as opp_id is deprecated
 */
static int __deprecated opp_to_freq(unsigned long *freq, enum opp_t opp_type,
				 u8 opp_id)
{
	struct omap_opp *opp;

	BUG_ON(!freq || opp_type >= OPP_TYPES_MAX);

	opp = opp_find_by_opp_id(opp_type, opp_id);
	if (IS_ERR(opp))
		return -EINVAL;

	*freq = opp_get_freq(opp);

	return 0;
}

/**
 * freq_to_opp - convert a frequency back to OPP ID (DEPRECATED)
 * @opp_id: opp ID returned back to caller
 * @opp_type: OPP type where we need to look.
 * @freq: frequency we are searching for
 *
 * return 0 and opp_id is populated if we find the freq, else, we return error
 *
 * NOTE: this function is a standin for the timebeing as opp_id is deprecated
 */
static int __deprecated freq_to_opp(u8 *opp_id, enum opp_t opp_type,
		unsigned long freq)
{
	struct omap_opp *opp;

	BUG_ON(opp_type >= OPP_TYPES_MAX);
	opp = opp_find_freq_ceil(opp_type, &freq);
	if (IS_ERR(opp))
		return -EINVAL;
	*opp_id = opp_get_opp_id(opp);
	return 0;
}

/**
 * init_opp - Initialize the OPP resource
 */
void init_opp(struct shared_resource *resp)
{
	struct clk *l3_clk;
	int ret;
	u8 opp_id;
	resp->no_of_users = 0;

	/* Initialize the current level of the OPP resource
	* to the  opp set by u-boot.
	*/
	if (strcmp(resp->name, "vdd1_opp") == 0) {
		vdd1_resp = resp;
		dpll1_clk = clk_get(NULL, "dpll1_ck");
		dpll2_clk = clk_get(NULL, "dpll2_ck");
		ret = freq_to_opp(&opp_id, OPP_MPU, dpll1_clk->rate);
		if (ret) {
			pr_err("%s: initializing %s failed! !match for %ld\n",
				__func__, resp->name, dpll1_clk->rate);
			if (dpll1_clk)
				clk_put(dpll1_clk);
			if (dpll2_clk)
				clk_put(dpll2_clk);
			dpll1_clk = NULL;
			dpll2_clk = NULL;
			vdd1_resp = NULL;
			return;
		}
		curr_vdd1_opp = opp_id;
	} else if (strcmp(resp->name, "vdd2_opp") == 0) {
		vdd2_resp = resp;
		dpll3_clk = clk_get(NULL, "dpll3_m2_ck");
		l3_clk = clk_get(NULL, "l3_ick");
		ret = freq_to_opp(&opp_id, OPP_L3, l3_clk->rate);
		if (ret) {
			pr_err("%s: initializing %s failed! !match for %ld\n",
				__func__, resp->name, l3_clk->rate);
			if (l3_clk)
				clk_put(l3_clk);
			if (dpll3_clk)
				clk_put(dpll3_clk);
			l3_clk = NULL;
			dpll3_clk = NULL;
			vdd2_resp = NULL;
			return;
		}
		curr_vdd2_opp = opp_id;
	}
	resp->curr_level = opp_id;
	return;
}

int resource_access_opp_lock(int res, int delta)
{
	if (res == VDD1_OPP) {
		vdd1_lock += delta;
		return vdd1_lock;
	} else if (res == VDD2_OPP) {
		vdd2_lock += delta;
		return vdd2_lock;
	}
	return -EINVAL;
}

#ifndef CONFIG_CPU_FREQ
static unsigned long compute_lpj(unsigned long ref, u_int div, u_int mult)
{
	unsigned long new_jiffy_l, new_jiffy_h;

	/*
	 * Recalculate loops_per_jiffy.  We do it this way to
	 * avoid math overflow on 32-bit machines.  Maybe we
	 * should make this architecture dependent?  If you have
	 * a better way of doing this, please replace!
	 *
	 *    new = old * mult / div
	 */
	new_jiffy_h = ref / div;
	new_jiffy_l = (ref % div) / 100;
	new_jiffy_h *= mult;
	new_jiffy_l = new_jiffy_l * mult / div;

	return new_jiffy_h + new_jiffy_l * 100;
}
#endif

static int program_opp_freq(int res, int target_level, int current_level)
{
	int ret = 0, l3_div;
	int *curr_opp;
	unsigned long mpu_freq, dsp_freq, l3_freq;
#ifndef CONFIG_CPU_FREQ
	unsigned long mpu_cur_freq;
#endif

	/* Check if I can actually switch or not */
	if (res == VDD1_OPP) {
		ret = opp_to_freq(&mpu_freq, OPP_MPU, target_level);
		ret |= opp_to_freq(&dsp_freq, OPP_DSP, target_level);
#ifndef CONFIG_CPU_FREQ
		ret |= opp_to_freq(&mpu_cur_freq, OPP_MPU, current_level);
#endif
	} else {
		ret = opp_to_freq(&l3_freq, OPP_L3, target_level);
	}
	/* we would have caught all bad levels earlier.. */
	if (unlikely(ret))
		return ret;

	lock_scratchpad_sem();
	if (res == VDD1_OPP) {
		curr_opp = &curr_vdd1_opp;
		clk_set_rate(dpll1_clk, mpu_freq);
		clk_set_rate(dpll2_clk, dsp_freq);
#ifndef CONFIG_CPU_FREQ
		/*Update loops_per_jiffy if processor speed is being changed*/
		loops_per_jiffy = compute_lpj(loops_per_jiffy,
			mpu_cur_freq / 1000, mpu_freq / 1000);
#endif
	} else {
		curr_opp = &curr_vdd2_opp;
		l3_div = cm_read_mod_reg(CORE_MOD, CM_CLKSEL) &
			OMAP3430_CLKSEL_L3_MASK;
		ret = clk_set_rate(dpll3_clk, l3_freq * l3_div);
	}
	if (ret) {
		unlock_scratchpad_sem();
		return current_level;
	}
#ifdef CONFIG_PM
	omap3_save_scratchpad_contents();
#endif
	unlock_scratchpad_sem();

	*curr_opp = target_level;
	return target_level;
}

static int program_opp(int res, enum opp_t opp_type, int target_level,
		int current_level)
{
	int i, ret = 0, raise;
	unsigned long freq;
#ifdef CONFIG_OMAP_SMARTREFLEX
	unsigned long t_opp, c_opp;

	t_opp = ID_VDD(res) | ID_OPP_NO(target_level);
	c_opp = ID_VDD(res) | ID_OPP_NO(current_level);
#endif

	/* See if have a freq associated, if not, invalid opp */
	ret = opp_to_freq(&freq, opp_type, target_level);
	if (unlikely(ret))
		return ret;

	if (target_level > current_level)
		raise = 1;
	else
		raise = 0;

	for (i = 0; i < 2; i++) {
		if (i == raise)
			ret = program_opp_freq(res, target_level,
					current_level);
#ifdef CONFIG_OMAP_SMARTREFLEX
		else {
			u8 vc, vt;
			struct omap_opp *oppx;
			unsigned long uvdc;

			/*
			 * transitioning from good to good OPP
			 * none of the following should fail..
			 */
			oppx = opp_find_freq_exact(opp_type, freq, true);
			BUG_ON(IS_ERR(oppx));
			uvdc = opp_get_voltage(oppx);
			vt = omap_twl_uv_to_vsel(uvdc);

			BUG_ON(opp_to_freq(&freq, opp_type, current_level));
			oppx = opp_find_freq_exact(opp_type, freq, true);
			BUG_ON(IS_ERR(oppx));
			uvdc = opp_get_voltage(oppx);
			vc = omap_twl_uv_to_vsel(uvdc);

			/* ok to scale.. */
			sr_voltagescale_vcbypass(t_opp, c_opp, vt, vc);
		}
#endif
	}

	return ret;
}

int resource_set_opp_level(int res, u32 target_level, int flags)
{
	unsigned long mpu_freq, mpu_old_freq, l3_freq;
	int ret;
#ifdef CONFIG_CPU_FREQ
	struct cpufreq_freqs freqs_notify;
#endif
	struct shared_resource *resp;

	if (res == VDD1_OPP)
		resp = vdd1_resp;
	else if (res == VDD2_OPP)
		resp = vdd2_resp;
	else
		return 0;

	if (resp->curr_level == target_level)
		return 0;

	/* Check if I can actually switch or not */
	if (res == VDD1_OPP) {
		ret = opp_to_freq(&mpu_freq, OPP_MPU, target_level);
		ret |= opp_to_freq(&mpu_old_freq, OPP_MPU, resp->curr_level);
	} else {
		ret = opp_to_freq(&l3_freq, OPP_L3, target_level);
	}
	if (ret)
		return ret;

	mutex_lock(&dvfs_mutex);

	if (res == VDD1_OPP) {
		if (flags != OPP_IGNORE_LOCK && vdd1_lock) {
			mutex_unlock(&dvfs_mutex);
			return 0;
		}
#ifdef CONFIG_CPU_FREQ
		freqs_notify.old = mpu_old_freq/1000;
		freqs_notify.new = mpu_freq/1000;
		freqs_notify.cpu = 0;
		/* Send pre notification to CPUFreq */
		cpufreq_notify_transition(&freqs_notify, CPUFREQ_PRECHANGE);
#endif
		resp->curr_level = program_opp(res, OPP_MPU, target_level,
			resp->curr_level);
#ifdef CONFIG_CPU_FREQ
		/* Send a post notification to CPUFreq */
		cpufreq_notify_transition(&freqs_notify, CPUFREQ_POSTCHANGE);
#endif
	} else {
		if (!(flags & OPP_IGNORE_LOCK) && vdd2_lock) {
			mutex_unlock(&dvfs_mutex);
			return 0;
		}
		resp->curr_level = program_opp(res, OPP_L3, target_level,
			resp->curr_level);
	}
	mutex_unlock(&dvfs_mutex);
	return 0;
}

int set_opp(struct shared_resource *resp, u32 target_level)
{
	int ret = -EINVAL;

	if (resp == vdd1_resp) {
		if (target_level < 3)
			resource_release("vdd2_opp", &vdd2_dev);

		ret = resource_set_opp_level(VDD1_OPP, target_level, 0);
		/*
		 * For VDD1 OPP3 and above, make sure the interconnect
		 * is at 100Mhz or above.
		 * throughput in KiB/s for 100 Mhz = 100 * 1000 * 4.
		 */
		if (target_level >= 3)
			resource_request("vdd2_opp", &vdd2_dev, 400000);

	} else if (resp == vdd2_resp) {
		unsigned long req_l3_freq;
		struct omap_opp *oppx = NULL;

		/* Convert the tput in KiB/s to Bus frequency in MHz */
		req_l3_freq = (target_level * 1000)/4;

		/* Do I have a best match? */
		oppx = opp_find_freq_ceil(OPP_L3, &req_l3_freq);
		if (IS_ERR(oppx)) {
			/* Give me the best we got */
			req_l3_freq = ULONG_MAX;
			oppx = opp_find_freq_floor(OPP_L3, &req_l3_freq);
		}

		/* uh uh.. no OPPs?? */
		BUG_ON(IS_ERR(oppx));

		target_level = 0;
		ret = freq_to_opp((u8 *)&target_level, OPP_L3, req_l3_freq);
		/* we dont expect this to fail */
		BUG_ON(ret);

		ret = resource_set_opp_level(VDD2_OPP, target_level, 0);
	}
	return 0;
}

/**
 * validate_opp - Validates if valid VDD1 OPP's are passed as the
 * target_level.
 * VDD2 OPP levels are passed as L3 throughput, which are then mapped
 * to an appropriate OPP.
 */
int validate_opp(struct shared_resource *resp, u32 target_level)
{
	unsigned long x;
	if (strcmp(resp->name, "mpu_freq") == 0)
		return opp_to_freq(&x, OPP_MPU, target_level);
	else if (strcmp(resp->name, "dsp_freq") == 0)
		return opp_to_freq(&x, OPP_DSP, target_level);
	return 0;
}

/**
 * init_freq - Initialize the frequency resource.
 */
void init_freq(struct shared_resource *resp)
{
	char *linked_res_name;
	int ret = -EINVAL;
	unsigned long freq = 0;
	resp->no_of_users = 0;

	linked_res_name = (char *)resp->resource_data;
	/* Initialize the current level of the Freq resource
	* to the frequency set by u-boot.
	*/
	if (strcmp(resp->name, "mpu_freq") == 0)
		/* MPU freq in Mhz */
		ret = opp_to_freq(&freq, OPP_MPU, curr_vdd1_opp);
	else if (strcmp(resp->name, "dsp_freq") == 0)
		/* DSP freq in Mhz */
		ret = opp_to_freq(&freq, OPP_DSP, curr_vdd1_opp);
	if (ret)
		pr_err("%s: initializing frequency failed!\n", __func__);

	resp->curr_level = freq;
	return;
}

int set_freq(struct shared_resource *resp, u32 target_level)
{
	u8 vdd1_opp;
	int ret = -EINVAL;

	if (strcmp(resp->name, "mpu_freq") == 0) {
		ret = freq_to_opp(&vdd1_opp, OPP_MPU, target_level);
		if (!ret)
			ret = resource_request("vdd1_opp", &dummy_mpu_dev,
					vdd1_opp);
	} else if (strcmp(resp->name, "dsp_freq") == 0) {
		ret = freq_to_opp(&vdd1_opp, OPP_DSP, target_level);
		if (!ret)
			ret = resource_request("vdd1_opp", &dummy_dsp_dev,
					vdd1_opp);
	}
	if (!ret)
		resp->curr_level = target_level;
	return ret;
}

int validate_freq(struct shared_resource *resp, u32 target_level)
{
	u8 x;
	if (strcmp(resp->name, "mpu_freq") == 0)
		return freq_to_opp(&x, OPP_MPU, target_level);
	else if (strcmp(resp->name, "dsp_freq") == 0)
		return freq_to_opp(&x, OPP_DSP, target_level);
	return 0;
}
