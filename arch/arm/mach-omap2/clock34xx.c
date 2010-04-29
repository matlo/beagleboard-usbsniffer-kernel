/*
 * OMAP3-specific clock framework functions
 *
 * Copyright (C) 2007-2008 Texas Instruments, Inc.
 * Copyright (C) 2007-2009 Nokia Corporation
 *
 * Written by Paul Walmsley
 * Testing and integration fixes by Jouni Högander
 *
 * Parts of this code are based on code written by
 * Richard Woodruff, Tony Lindgren, Tuukka Tikkanen, Karthik Dasu
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#undef DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/cpufreq.h>

#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/sram.h>
#include <plat/sdrc.h>
#include <plat/omap-pm.h>

#include <asm/div64.h>
#include <asm/clkdev.h>

#include <plat/sdrc.h>
#include "clock.h"
#include "clock34xx.h"
#include "sdrc.h"
#include "prm.h"
#include "prm-regbits-34xx.h"
#include "cm.h"
#include "cm-regbits-34xx.h"
#include "omap3-opp.h"

#define CYCLES_PER_MHZ			1000000

/*
 * DPLL5_FREQ_FOR_USBHOST: USBHOST and USBTLL are the only clocks
 * that are sourced by DPLL5, and both of these require this clock
 * to be at 120 MHz for proper operation.
 */
#define DPLL5_FREQ_FOR_USBHOST		120000000

/* needed by omap3_core_dpll_m2_set_rate() */
struct clk *sdrc_ick_p, *arm_fck_p;

/*
 * VDD1 and VDD2 OPPs derived from the bootarg 'mpurate'
 */
unsigned int vdd1_opp = 0;
unsigned int vdd2_opp = 0;

/**
 * omap3430es2_clk_ssi_find_idlest - return CM_IDLEST info for SSI
 * @clk: struct clk * being enabled
 * @idlest_reg: void __iomem ** to store CM_IDLEST reg address into
 * @idlest_bit: pointer to a u8 to store the CM_IDLEST bit shift into
 *
 * The OMAP3430ES2 SSI target CM_IDLEST bit is at a different shift
 * from the CM_{I,F}CLKEN bit.  Pass back the correct info via
 * @idlest_reg and @idlest_bit.  No return value.
 */
static void omap3430es2_clk_ssi_find_idlest(struct clk *clk,
					    void __iomem **idlest_reg,
					    u8 *idlest_bit)
{
	u32 r;

	r = (((__force u32)clk->enable_reg & ~0xf0) | 0x20);
	*idlest_reg = (__force void __iomem *)r;
	*idlest_bit = OMAP3430ES2_ST_SSI_IDLE_SHIFT;
}

const struct clkops clkops_omap3430es2_ssi_wait = {
	.enable		= omap2_dflt_clk_enable,
	.disable	= omap2_dflt_clk_disable,
	.find_idlest	= omap3430es2_clk_ssi_find_idlest,
	.find_companion = omap2_clk_dflt_find_companion,
};

/**
 * omap3430es2_clk_dss_usbhost_find_idlest - CM_IDLEST info for DSS, USBHOST
 * @clk: struct clk * being enabled
 * @idlest_reg: void __iomem ** to store CM_IDLEST reg address into
 * @idlest_bit: pointer to a u8 to store the CM_IDLEST bit shift into
 *
 * Some OMAP modules on OMAP3 ES2+ chips have both initiator and
 * target IDLEST bits.  For our purposes, we are concerned with the
 * target IDLEST bits, which exist at a different bit position than
 * the *CLKEN bit position for these modules (DSS and USBHOST) (The
 * default find_idlest code assumes that they are at the same
 * position.)  No return value.
 */
static void omap3430es2_clk_dss_usbhost_find_idlest(struct clk *clk,
						    void __iomem **idlest_reg,
						    u8 *idlest_bit)
{
	u32 r;

	r = (((__force u32)clk->enable_reg & ~0xf0) | 0x20);
	*idlest_reg = (__force void __iomem *)r;
	/* USBHOST_IDLE has same shift */
	*idlest_bit = OMAP3430ES2_ST_DSS_IDLE_SHIFT;
}

const struct clkops clkops_omap3430es2_dss_usbhost_wait = {
	.enable		= omap2_dflt_clk_enable,
	.disable	= omap2_dflt_clk_disable,
	.find_idlest	= omap3430es2_clk_dss_usbhost_find_idlest,
	.find_companion = omap2_clk_dflt_find_companion,
};

/**
 * omap3430es2_clk_hsotgusb_find_idlest - return CM_IDLEST info for HSOTGUSB
 * @clk: struct clk * being enabled
 * @idlest_reg: void __iomem ** to store CM_IDLEST reg address into
 * @idlest_bit: pointer to a u8 to store the CM_IDLEST bit shift into
 *
 * The OMAP3430ES2 HSOTGUSB target CM_IDLEST bit is at a different
 * shift from the CM_{I,F}CLKEN bit.  Pass back the correct info via
 * @idlest_reg and @idlest_bit.  No return value.
 */
static void omap3430es2_clk_hsotgusb_find_idlest(struct clk *clk,
						 void __iomem **idlest_reg,
						 u8 *idlest_bit)
{
	u32 r;

	r = (((__force u32)clk->enable_reg & ~0xf0) | 0x20);
	*idlest_reg = (__force void __iomem *)r;
	*idlest_bit = OMAP3430ES2_ST_HSOTGUSB_IDLE_SHIFT;
}

const struct clkops clkops_omap3430es2_hsotgusb_wait = {
	.enable		= omap2_dflt_clk_enable,
	.disable	= omap2_dflt_clk_disable,
	.find_idlest	= omap3430es2_clk_hsotgusb_find_idlest,
	.find_companion = omap2_clk_dflt_find_companion,
};

/** omap3_pwrdn_clk_enable_with_hsdiv_restore - enable clocks suffering
 *         from HSDivider problem.
 * @clk: DPLL output struct clk
 *
 * 3630 only: dpll3_m3_ck, dpll4_m2_ck, dpll4_m3_ck, dpll4_m4_ck, dpll4_m5_ck
 * & dpll4_m6_ck dividers get lost after their respective PWRDN bits are set.
 * Any write to the corresponding CM_CLKSEL register will refresh the
 * dividers.  Only x2 clocks are affected, so it is safe to trust the parent
 * clock information to refresh the CM_CLKSEL registers.
 */
int omap3_pwrdn_clk_enable_with_hsdiv_restore(struct clk *clk)
{
	u32 orig_v, v, c, clksel_shift, max_div;
	int ret;

	/* enable the clock */
	ret = omap2_dflt_clk_enable(clk);

	/* Restore the dividers */
	if (!ret) {
		v = __raw_readl(clk->parent->clksel_reg);
		orig_v = v;

		clksel_shift = __ffs(clk->parent->clksel_mask);

		max_div = clk->parent->clksel_mask >> clksel_shift;

		/* Isolate the current divider */
		c = v & clk->parent->clksel_mask;
		c >>= clksel_shift;

		/* Prevent excessively high clock rates if divider would wrap */
		c += (c == max_div) ? -1 : 1;

		/* Write the temporarily altered divider back */
		c <<= clksel_shift;
		v &= ~c;
		v |= c;
		__raw_writel(v, clk->parent->clksel_reg);

		/* Write the original divider */
		__raw_writel(orig_v, clk->parent->clksel_reg);
	}

	return ret;
}

const struct clkops clkops_omap3_pwrdn_with_hsdiv_wait_restore = {
	.enable		= omap3_pwrdn_clk_enable_with_hsdiv_restore,
	.disable	= omap2_dflt_clk_disable,
	.find_companion	= omap2_clk_dflt_find_companion,
	.find_idlest	= omap2_clk_dflt_find_idlest,
};

const struct clkops clkops_noncore_dpll_ops = {
	.enable		= omap3_noncore_dpll_enable,
	.disable	= omap3_noncore_dpll_disable,
};

int omap3_dpll4_set_rate(struct clk *clk, unsigned long rate)
{
	/*
	 * According to the 12-5 CDP code from TI, "Limitation 2.5"
	 * on 3430ES1 prevents us from changing DPLL multipliers or dividers
	 * on DPLL4.
	 */
	if (!cpu_is_omap3630() && cpu_is_omap34xx() && omap_rev_is_1_0()) {
		printk(KERN_ERR "clock: DPLL4 cannot change rate due to "
		       "silicon 'Limitation 2.5' on 3430ES1.\n");
		return -EINVAL;
	}
	return omap3_noncore_dpll_set_rate(clk, rate);
}


/*
 * CORE DPLL (DPLL3) rate programming functions
 *
 * These call into SRAM code to do the actual CM writes, since the SDRAM
 * is clocked from DPLL3.
 */

/**
 * omap3_core_dpll_m2_set_rate - set CORE DPLL M2 divider
 * @clk: struct clk * of DPLL to set
 * @rate: rounded target rate
 *
 * Program the DPLL M2 divider with the rounded target rate.  Returns
 * -EINVAL upon error, or 0 upon success.
 */
int omap3_core_dpll_m2_set_rate(struct clk *clk, unsigned long rate)
{
	u32 new_div = 0;
	u32 unlock_dll = 0;
	u32 c;
	unsigned long validrate, sdrcrate, _mpurate;
	struct omap_sdrc_params *sdrc_cs0;
	struct omap_sdrc_params *sdrc_cs1;
	int ret;

	if (!clk || !rate)
		return -EINVAL;

	validrate = omap2_clksel_round_rate_div(clk, rate, &new_div);
	if (validrate != rate)
		return -EINVAL;

	sdrcrate = sdrc_ick_p->rate;
	if (rate > clk->rate)
		sdrcrate <<= ((rate / clk->rate) >> 1);
	else
		sdrcrate >>= ((clk->rate / rate) >> 1);

	ret = omap2_sdrc_get_params(sdrcrate, &sdrc_cs0, &sdrc_cs1);
	if (ret)
		return -EINVAL;

	if (sdrcrate < MIN_SDRC_DLL_LOCK_FREQ) {
		pr_debug("clock: will unlock SDRC DLL\n");
		unlock_dll = 1;
	}

	/*
	 * XXX This only needs to be done when the CPU frequency changes
	 */
	_mpurate = arm_fck_p->rate / CYCLES_PER_MHZ;
	c = (_mpurate << SDRC_MPURATE_SCALE) >> SDRC_MPURATE_BASE_SHIFT;
	c += 1;  /* for safety */
	c *= SDRC_MPURATE_LOOPS;
	c >>= SDRC_MPURATE_SCALE;
	if (c == 0)
		c = 1;

	pr_debug("clock: changing CORE DPLL rate from %lu to %lu\n", clk->rate,
		 validrate);
	pr_debug("clock: SDRC CS0 timing params used:"
		 " RFR %08x CTRLA %08x CTRLB %08x MR %08x\n",
		 sdrc_cs0->rfr_ctrl, sdrc_cs0->actim_ctrla,
		 sdrc_cs0->actim_ctrlb, sdrc_cs0->mr);
	if (sdrc_cs1)
		pr_debug("clock: SDRC CS1 timing params used: "
		 " RFR %08x CTRLA %08x CTRLB %08x MR %08x\n",
		 sdrc_cs1->rfr_ctrl, sdrc_cs1->actim_ctrla,
		 sdrc_cs1->actim_ctrlb, sdrc_cs1->mr);

	if (sdrc_cs1)
		omap3_configure_core_dpll(
				  new_div, unlock_dll, c, rate > clk->rate,
				  sdrc_cs0->rfr_ctrl, sdrc_cs0->actim_ctrla,
				  sdrc_cs0->actim_ctrlb, sdrc_cs0->mr,
				  sdrc_cs1->rfr_ctrl, sdrc_cs1->actim_ctrla,
				  sdrc_cs1->actim_ctrlb, sdrc_cs1->mr);
	else
		omap3_configure_core_dpll(
				  new_div, unlock_dll, c, rate > clk->rate,
				  sdrc_cs0->rfr_ctrl, sdrc_cs0->actim_ctrla,
				  sdrc_cs0->actim_ctrlb, sdrc_cs0->mr,
				  0, 0, 0, 0);

	return 0;
}

/* Common clock code */

/*
 * As it is structured now, this will prevent an OMAP2/3 multiboot
 * kernel from compiling.  This will need further attention.
 */
#if defined(CONFIG_ARCH_OMAP3)

#ifdef CONFIG_CPU_FREQ
static struct cpufreq_frequency_table freq_table[MAX_VDD1_OPP+1];

void omap2_clk_init_cpufreq_table(struct cpufreq_frequency_table **table)
{
	struct omap_opp *prcm;
	int i = 0;

	if (!mpu_opps)
		return;

	prcm = mpu_opps + get_max_vdd1();;
	for (; prcm->rate; prcm--) {
		freq_table[i].index = i;
		freq_table[i].frequency = prcm->rate / 1000;
		i++;
	}

	if (i == 0) {
		printk(KERN_WARNING "%s: failed to initialize frequency \
								table\n",
								__func__);
		return;
	}

	freq_table[i].index = i;
	freq_table[i].frequency = CPUFREQ_TABLE_END;

	*table = &freq_table[0];
}
#endif

struct clk_functions omap2_clk_functions = {
	.clk_enable		= omap2_clk_enable,
	.clk_disable		= omap2_clk_disable,
	.clk_round_rate		= omap2_clk_round_rate,
	.clk_set_rate		= omap2_clk_set_rate,
	.clk_set_parent		= omap2_clk_set_parent,
	.clk_disable_unused	= omap2_clk_disable_unused,
#ifdef CONFIG_CPU_FREQ
	.clk_init_cpufreq_table = omap2_clk_init_cpufreq_table,
#endif
};

/*
 * Set clocks for bypass mode for reboot to work.
 */
void omap2_clk_prepare_for_reboot(void)
{
	/* REVISIT: Not ready for 343x */
#if 0
	u32 rate;

	if (vclk == NULL || sclk == NULL)
		return;

	rate = clk_get_rate(sclk);
	clk_set_rate(vclk, rate);
#endif
}

void omap3_clk_lock_dpll5(void)
{
	struct clk *dpll5_clk;
	struct clk *dpll5_m2_clk;

	dpll5_clk = clk_get(NULL, "dpll5_ck");
	clk_set_rate(dpll5_clk, DPLL5_FREQ_FOR_USBHOST);
	clk_enable(dpll5_clk);

	/* Enable autoidle to allow it to enter low power bypass */
	omap3_dpll_allow_idle(dpll5_clk);

	/* Program dpll5_m2_clk divider for no division */
	dpll5_m2_clk = clk_get(NULL, "dpll5_m2_ck");
	clk_enable(dpll5_m2_clk);
	clk_set_rate(dpll5_m2_clk, DPLL5_FREQ_FOR_USBHOST);

	clk_disable(dpll5_m2_clk);
	clk_disable(dpll5_clk);
	return;
}

extern void __init omap3_iva_idle(void);

/*
 * Initialize IVA to a idle state. This is typically done by the
 * pm code, but that happnes during the late init stages. We need this
 * to be done before the IVA clock rates are switched during boot.
 */
static void __init omap2_clk_iva_init_to_idle(void)
{
	cm_write_mod_reg(1 << OMAP3430_AUTO_IVA2_DPLL_SHIFT,
			OMAP3430_IVA2_MOD, CM_AUTOIDLE2);

	/* Don't attach IVA interrupts */
	prm_write_mod_reg(0, WKUP_MOD, OMAP3430_PM_IVAGRPSEL);
	prm_write_mod_reg(0, CORE_MOD, OMAP3430_PM_IVAGRPSEL1);
	prm_write_mod_reg(0, CORE_MOD, OMAP3430ES2_PM_IVAGRPSEL3);
	prm_write_mod_reg(0, OMAP3430_PER_MOD, OMAP3430_PM_IVAGRPSEL);

	omap3_iva_idle();

}

/* REVISIT: Move this init stuff out into clock.c */

/*
 * Switch the MPU rate if specified on cmdline.
 * We cannot do this early until cmdline is parsed.
 *
 * Beyond a certain limit, frequency cannot be increased without corresponding
 * increase in voltage. This tolerance varies across processors and, possibly,
 * their revisions.
 * This function only derives the target OPP based on bootarg 'mpurate'.
 * Actual change in the frequency is deferred until the voltage corresponding
 * to the OPP has been set.
 */
static int __init omap2_clk_arch_init(void)
{
	struct omap_opp *opp_table;
	short valid=0, err=0, i;

	if (!mpurate)
		return -EINVAL;

	/*
	 * Check if OPP tables are defined.
	 */
	if (WARN((!mpu_opps), "OPP table not defined for MPU\n"))
		err = 1;

	if (omap3_has_iva())
		if (WARN((!dsp_opps), "OPP table not defined for DSP\n"))
			err = 1;

	if (WARN((!l3_opps), "OPP table not defined for L3\n"))
		err = 1;

	if (err)
		return -ENOENT;

	/*
	 * Check if silicon supports specified mpurate.
	 * Else, use the next highest.
	 */
	if (!cpu_is_omap3630() && (mpurate == S720M) && !omap3_has_720m()) {
		mpurate = S600M;
		pr_err("This silicon doesn't support 720MHz\n");
	}

	/*
	 * Select VDD1 OPP corresponding to mpurate
	 */
	opp_table = mpu_opps;

	for (i = 1; opp_table[i].opp_id <= get_max_vdd1(); i++) {
		if (opp_table[i].rate == mpurate) {
			valid = 1;
			break;
		}
	}

	if (valid) {
		vdd1_opp = opp_table[i].opp_id;
	} else {
		pr_err("Invalid MPU rate (%u)\n", mpurate);
		return -EINVAL;
	}

	/*
	 * Match lowest OPP setting for VDD1 with lowest OPP for VDD2 as well.
	 */
	if (vdd1_opp == VDD1_OPP1) {
		if (cpu_is_omap3630())
			vdd2_opp = VDD2_OPP1;
		else
			vdd2_opp = VDD2_OPP2;
	} else {
		vdd2_opp = VDD2_OPP2;
	}

	pr_info("Target VDD1 OPP = %d, VDD2 OPP = %d\n", vdd1_opp, vdd2_opp);

	return 0;
}
arch_initcall(omap2_clk_arch_init);


int __init omap2_clk_set_freq(void)
{
	struct clk *osc_sys_ck, *dpll1_ck, *arm_fck, *core_ck;
	struct clk *dpll2_ck, *iva2_ck, *dpll3_m2_ck;
	unsigned long osc_sys_rate;
	unsigned long dsprate, l3rate;
	short err = 0 ;
	int l3div;

	/*
	 * Check if any processing is required.
	 */
	if ((vdd1_opp == 0) && (vdd2_opp == 0))
		return 0;

	if (WARN((vdd1_opp == 0), "VDD1 OPP is not set.\n"))
		err = 1;

	if (WARN((vdd2_opp == 0), "VDD2 OPP is not set.\n"))
		err = 1;

	/*
	 * Attempt to get the required clocks
	 */
	dpll1_ck = clk_get(NULL, "dpll1_ck");
	if (WARN(IS_ERR(dpll1_ck), "Failed to get dpll1_ck.\n"))
		err = 1;

	arm_fck = clk_get(NULL, "arm_fck");
	if (WARN(IS_ERR(arm_fck), "Failed to get arm_fck.\n"))
		err = 1;

	core_ck = clk_get(NULL, "core_ck");
	if (WARN(IS_ERR(core_ck), "Failed to get core_ck.\n"))
		err = 1;

	osc_sys_ck = clk_get(NULL, "osc_sys_ck");
	if (WARN(IS_ERR(osc_sys_ck), "Failed to get osc_sys_ck.\n"))
		err = 1;

	dpll2_ck = clk_get(NULL, "dpll2_ck");
	if (WARN(IS_ERR("dpll2_ck"), "Failed to get dpll2_ck.\n"))
		err = 1;

	iva2_ck = clk_get(NULL, "iva2_ck");
	if (WARN(IS_ERR("iva2_ck"), "Failed to get iva2_ck.\n"))
		err = 1;

	dpll3_m2_ck = clk_get(NULL, "dpll3_m2_ck");
	if (WARN(IS_ERR("dpll3_m2_ck"), "Failed to get dpll3_m2_ck.\n"))
		err = 1;

	if (err)
		return -ENOENT;

	/*
	 * Set MPU frequency
	 */
	mpurate = mpu_opps [vdd1_opp].rate;
	if (clk_set_rate(dpll1_ck, mpurate))
		pr_err("Unable to set MPU frequency (%lu)\n", mpurate);

	/*
	 * Set DSP frequency
	 */
	if (omap3_has_iva()) {
		omap2_clk_iva_init_to_idle();

		dsprate = dsp_opps [vdd1_opp].rate;
		if (clk_set_rate(dpll2_ck, dsprate))
			pr_err("Unable to set DSP frequency (%lu)\n", dsprate);
	}

	/*
	 * Set L3 frequency
	 */
	l3div  = cm_read_mod_reg(CORE_MOD, CM_CLKSEL) &
			OMAP3430_CLKSEL_L3_MASK;

	l3rate = l3_opps[vdd2_opp].rate * l3div;

	if (clk_set_rate(dpll3_m2_ck, l3rate))
		pr_err("Unable to set L3 frequency (%lu)\n", l3rate);

	/*
	 * Re-calculate the clocks
	 */
	recalculate_root_clocks();

	osc_sys_rate = clk_get_rate(osc_sys_ck);

	pr_info("Switched to new clocking rate (Crystal/Core/MPU): "
		"%ld.%01ld/%ld/%ld MHz\n",
		(osc_sys_rate / 1000000),
		((osc_sys_rate / 100000) % 10),
		(clk_get_rate(core_ck) / 1000000),
		(clk_get_rate(arm_fck) / 1000000));

	if (omap3_has_iva()) {
		pr_info("IVA2 clocking rate: %ld MHz\n",
		       (clk_get_rate(iva2_ck) / 1000000)) ;
	}

	calibrate_delay();

	return 0;
}
#endif
