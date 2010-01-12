/*
 * linux/arch/arm/mach-omap2/mmc-am3517evm.c
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Author: Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <plat/control.h>
#include <plat/mmc.h>
#include <plat/board.h>
#include "mmc-am3517evm.h"

#define LDO_CLR			0x00
#define VSEL_S2_CLR		0x40

#define VMMC1_DEV_GRP		0x27
#define VMMC1_CLR		0x00
#define VMMC1_315V		0x03
#define VMMC1_300V		0x02
#define VMMC1_285V		0x01
#define VMMC1_185V		0x00
#define VMMC1_DEDICATED		0x2A

#define VMMC2_DEV_GRP		0x2B
#define VMMC2_CLR		0x40
#define VMMC2_315V		0x0c
#define VMMC2_300V		0x0b
#define VMMC2_285V		0x0a
#define VMMC2_260V		0x08
#define VMMC2_185V		0x06
#define VMMC2_DEDICATED		0x2E

#define VMMC_DEV_GRP_P1		0x20

#define HSMMC_NAME_LEN	9

#if defined(CONFIG_REGULATOR) || \
	(defined(CONFIG_MMC_OMAP) || defined(CONFIG_MMC_OMAP_MODULE) || \
	 defined(CONFIG_MMC_OMAP_HS) || defined(CONFIG_MMC_OMAP_HS_MODULE))

/*
 * MMC definitions
 *
 */
static struct mmc_controller {
	struct omap_mmc_platform_data	*mmc;
	u8		vmmc_dev_grp;
	u8		vmmc_dedicated;
	char		name[HSMMC_NAME_LEN];
} hsmmc[] = {
	{
		.vmmc_dev_grp		= VMMC1_DEV_GRP,
		.vmmc_dedicated		= VMMC1_DEDICATED,
	},
	{
		.vmmc_dev_grp		= VMMC2_DEV_GRP,
		.vmmc_dedicated		= VMMC2_DEDICATED,
	},
};

static int mmc_card_detect(int irq)
{
	unsigned i;

	for (i = 0; i < ARRAY_SIZE(hsmmc); i++) {
		struct omap_mmc_platform_data *mmc;

		mmc = hsmmc[i].mmc;
		if (!mmc)
			continue;
		if (irq != mmc->slots[0].card_detect_irq)
			continue;

		/* NOTE: assumes card detect signal is active-low */
		return !gpio_get_value_cansleep(mmc->slots[0].switch_pin);
	}
	return -ENOSYS;
}

static int mmc_get_ro(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	/* NOTE: assumes write protect signal is active-high */
	return gpio_get_value_cansleep(mmc->slots[0].gpio_wp);
}

/*
 * MMC Slot Initialization.
 */
static int mmc_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;
	int ret = 0;
	int i;

	ret = gpio_request(mmc->slots[0].switch_pin, "mmc_cd");
	if (ret)
		goto done;
	ret = gpio_direction_input(mmc->slots[0].switch_pin);
	if (ret)
		goto err;

	for (i = 0; i < ARRAY_SIZE(hsmmc); i++) {
		if (hsmmc[i].name == mmc->slots[0].name) {
			hsmmc[i].mmc = mmc;
			break;
		}
	}

	return 0;

err:
	gpio_free(mmc->slots[0].switch_pin);
done:
	mmc->slots[0].card_detect_irq = 0;
	mmc->slots[0].card_detect = NULL;

	dev_err(dev, "err %d configuring card detect\n", ret);
	return ret;
}

static void mmc_cleanup(struct device *dev)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	gpio_free(mmc->slots[0].switch_pin);
}

#ifdef CONFIG_PM

static int mmc_suspend(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	disable_irq(mmc->slots[0].card_detect_irq);
	return 0;
}

static int mmc_resume(struct device *dev, int slot)
{
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	enable_irq(mmc->slots[0].card_detect_irq);
	return 0;
}

#else
#define mmc_suspend	NULL
#define mmc_resume	NULL
#endif

/*
 * the MMC power setting function
 */

static int mmc1_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	return 0;
}

static int mmc2_set_power(struct device *dev, int slot, int power_on, int vdd)
{
	return 0;
}

static struct omap_mmc_platform_data *hsmmc_data[OMAP34XX_NR_MMC] __initdata;

void __init am3517_mmc_init(struct am3517_hsmmc_info *controllers)
{
	struct am3517_hsmmc_info *c;
	int nr_hsmmc = ARRAY_SIZE(hsmmc_data);

	for (c = controllers; c->mmc; c++) {
		struct mmc_controller *mmc_control = hsmmc + c->mmc - 1;
		struct omap_mmc_platform_data *mmc = hsmmc_data[c->mmc - 1];

		if (!c->mmc || c->mmc > nr_hsmmc) {
			pr_debug("MMC%d: no such controller\n", c->mmc);
			continue;
		}
		if (mmc) {
			pr_debug("MMC%d: already configured\n", c->mmc);
			continue;
		}

		mmc = kzalloc(sizeof(struct omap_mmc_platform_data), GFP_KERNEL);
		if (!mmc) {
			pr_err("Cannot allocate memory for mmc device!\n");
			return;
		}

		sprintf(mmc_control->name, "mmc%islot%i", c->mmc, 1);
		mmc->slots[0].name = mmc_control->name;
		mmc->nr_slots = 1;
		mmc->slots[0].ocr_mask = MMC_VDD_165_195 |
					MMC_VDD_26_27 | MMC_VDD_27_28 |
					MMC_VDD_29_30 |
					MMC_VDD_30_31 | MMC_VDD_31_32;
		mmc->slots[0].wires = c->wires;
		mmc->slots[0].internal_clock = !c->ext_clock;
		mmc->dma_mask = 0xffffffff;

		if (1) {
			mmc->init = mmc_late_init;
			mmc->cleanup = mmc_cleanup;
			mmc->suspend = mmc_suspend;
			mmc->resume = mmc_resume;

			mmc->slots[0].switch_pin = c->gpio_cd;
			mmc->slots[0].card_detect_irq = gpio_to_irq(c->gpio_cd);
			mmc->slots[0].card_detect = mmc_card_detect;
		} else
			mmc->slots[0].switch_pin = -EINVAL;

		/* write protect normally uses an OMAP gpio */
		if (gpio_is_valid(c->gpio_wp)) {
			gpio_request(c->gpio_wp, "mmc_wp");
			gpio_direction_input(c->gpio_wp);

			mmc->slots[0].gpio_wp = c->gpio_wp;
			mmc->slots[0].get_ro = mmc_get_ro;
		} else
			mmc->slots[0].gpio_wp = -EINVAL;

		/* NOTE:  we assume OMAP's MMC1 and MMC2 use
		 * the TWL4030's VMMC1 and VMMC2, respectively;
		 * and that OMAP's MMC3 isn't used.
		 */

		switch (c->mmc) {
		case 1:
			mmc->slots[0].set_power = mmc1_set_power;
			break;
		case 2:
			mmc->slots[0].set_power = mmc2_set_power;
			break;
		default:
			pr_err("MMC%d configuration not supported!\n", c->mmc);
			continue;
		}
		hsmmc_data[c->mmc - 1] = mmc;
	}

	omap2_init_mmc(hsmmc_data, OMAP34XX_NR_MMC);
}
#else
inline void am3517_mmc_init(struct am3517_hsmmc_info *info)
{
}
#endif
