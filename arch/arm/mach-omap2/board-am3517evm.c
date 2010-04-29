/*
 * linux/arch/arm/mach-omap2/board-am3517evm.c
 *
 * Copyright (C) 2009 Texas Instruments Incorporated
 * Author: Ranjith Lohithakshan <ranjithl@ti.com>
 *
 * Based on mach-omap2/board-omap3evm.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as  published by the
 * Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 * whether express or implied; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/i2c/tsc2004.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/input.h>
#include <linux/tca6416_keypad.h>
#include <linux/davinci_emac.h>
#include <linux/i2c/pca953x.h>
#include <linux/regulator/machine.h>
#include <linux/can/platform/ti_hecc.h>

#include <mach/hardware.h>
#include <mach/am35xx.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/usb.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>

#include <media/tvp514x.h>
#include <media/ti-media/vpfe_capture.h>

#include "mmc-am3517evm.h"
#include "mux.h"

#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE        SZ_128K

static struct mtd_partition am3517evm_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
{
       .name           = "xloader-nand",
       .offset         = 0,
       .size           = 4*(SZ_128K),
       .mask_flags     = MTD_WRITEABLE
},
{
       .name           = "uboot-nand",
       .offset         = MTDPART_OFS_APPEND,
       .size           = 14*(SZ_128K),
       .mask_flags     = MTD_WRITEABLE
},
{
       .name           = "params-nand",
       .offset         = MTDPART_OFS_APPEND,
       .size           = 2*(SZ_128K)
},
{
       .name           = "linux-nand",
       .offset         = MTDPART_OFS_APPEND,
       .size           = 40*(SZ_128K)
},
{
       .name           = "jffs2-nand",
       .size           = MTDPART_SIZ_FULL,
       .offset         = MTDPART_OFS_APPEND,
},
};

static struct omap_nand_platform_data am3517evm_nand_data = {
       .parts          = am3517evm_nand_partitions,
       .nr_parts       = ARRAY_SIZE(am3517evm_nand_partitions),
       .nand_setup     = NULL,
       .dma_channel    = -1,           /* disable DMA in OMAP NAND driver */
       .dev_ready      = NULL,
};

static struct resource am3517evm_nand_resource = {
       .flags          = IORESOURCE_MEM,
};

static struct platform_device am3517evm_nand_device = {
       .name           = "omap2-nand",
       .id             = 0,
       .dev            = {
                       .platform_data  = &am3517evm_nand_data,
       },
       .num_resources  = 1,
       .resource       = &am3517evm_nand_resource,
};

void __init am3517evm_flash_init(void)
{
       u8 cs = 0;
       u8 nandcs = GPMC_CS_NUM + 1;
       u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

       while (cs < GPMC_CS_NUM) {
               u32 ret = 0;
               ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

               if ((ret & 0xC00) == 0x800) {
                       /* Found it!! */
                       if (nandcs > GPMC_CS_NUM)
                               nandcs = cs;
               }
               cs++;
       }
       if (nandcs > GPMC_CS_NUM) {
               printk(KERN_INFO "NAND: Unable to find configuration "
                       " in GPMC\n ");
               return;
       }

       if (nandcs < GPMC_CS_NUM) {
               am3517evm_nand_data.cs   = nandcs;
               am3517evm_nand_data.gpmc_cs_baseaddr = (void *)(gpmc_base_add +
                                       GPMC_CS0_BASE + nandcs*GPMC_CS_SIZE);
               am3517evm_nand_data.gpmc_baseaddr   = (void *) (gpmc_base_add);

               if (platform_device_register(&am3517evm_nand_device) < 0)
                       printk(KERN_ERR "Unable to register NAND device\n");

       }
}


#define AM35XX_EVM_PHY_MASK		(0xF)
#define AM35XX_EVM_MDIO_FREQUENCY    	(1000000)

static struct emac_platform_data am3517_evm_emac_pdata = {
	.phy_mask       = AM35XX_EVM_PHY_MASK,
	.mdio_max_freq  = AM35XX_EVM_MDIO_FREQUENCY,
	.rmii_en        = 1,
};

static int __init eth_addr_setup(char *str)
{
	int i;

	if(str == NULL)
		return 0;
	for(i = 0; i <  ETH_ALEN; i++)
		am3517_evm_emac_pdata.mac_addr[i] = simple_strtol(&str[i*3],
							(char **)NULL, 16);
	return 1;
}

/* Get MAC address from kernel boot parameter eth=AA:BB:CC:DD:EE:FF */
__setup("eth=", eth_addr_setup);

static struct resource am3517_emac_resources[] = {
	{
		.start  = AM35XX_IPSS_EMAC_BASE,
		.end    = AM35XX_IPSS_EMAC_BASE + 0x3FFFF,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.end    = INT_35XX_EMAC_C0_RXTHRESH_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_RX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_TX_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
	{
		.start  = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.end    = INT_35XX_EMAC_C0_MISC_PULSE_IRQ,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device am3517_emac_device = {
	.name           = "davinci_emac",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(am3517_emac_resources),
	.resource       = am3517_emac_resources,
};

static void am3517_enable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR | AM35XX_CPGMAC_C0_MISC_PULSE_CLR |
		AM35XX_CPGMAC_C0_RX_THRESH_CLR );
	omap_ctrl_writel(regval,AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static void am3517_disable_ethernet_int(void)
{
	u32 regval;

	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = (regval | AM35XX_CPGMAC_C0_RX_PULSE_CLR |
		AM35XX_CPGMAC_C0_TX_PULSE_CLR);
	omap_ctrl_writel(regval,AM35XX_CONTROL_LVL_INTR_CLEAR);
	regval = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

void am3517_evm_ethernet_init(struct emac_platform_data *pdata)
{
	unsigned int regval;

	pdata->ctrl_reg_offset          = AM35XX_EMAC_CNTRL_OFFSET;
	pdata->ctrl_mod_reg_offset      = AM35XX_EMAC_CNTRL_MOD_OFFSET;
	pdata->ctrl_ram_offset          = AM35XX_EMAC_CNTRL_RAM_OFFSET;
	pdata->mdio_reg_offset          = AM35XX_EMAC_MDIO_OFFSET;
	pdata->ctrl_ram_size            = AM35XX_EMAC_CNTRL_RAM_SIZE;
	pdata->version                  = EMAC_VERSION_2;
	pdata->hw_ram_addr              = AM35XX_EMAC_HW_RAM_ADDR;
	pdata->interrupt_enable 	= am3517_enable_ethernet_int;
	pdata->interrupt_disable 	= am3517_disable_ethernet_int;
	am3517_emac_device.dev.platform_data     = pdata;
	platform_device_register(&am3517_emac_device);

	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);
	regval = regval & (~(AM35XX_CPGMACSS_SW_RST));
	omap_ctrl_writel(regval,AM35XX_CONTROL_IP_SW_RESET);
	regval = omap_ctrl_readl(AM35XX_CONTROL_IP_SW_RESET);

	return ;
}



#define LCD_PANEL_PWR		176
#define LCD_PANEL_BKLIGHT_PWR	182
#define LCD_PANEL_PWM		181

static int lcd_enabled;
static int dvi_enabled;

static void __init am3517_evm_display_init(void)
{
#if defined(CONFIG_PANEL_SHARP_LQ043T1DG01) || \
		defined(CONFIG_PANEL_SHARP_LQ043T1DG01_MODULE)
	int r;

	omap_mux_init_gpio(LCD_PANEL_PWR, OMAP_PIN_INPUT_PULLUP);
	omap_mux_init_gpio(LCD_PANEL_BKLIGHT_PWR, OMAP_PIN_INPUT_PULLDOWN);
	omap_mux_init_gpio(LCD_PANEL_PWM, OMAP_PIN_INPUT_PULLDOWN);
	/*
	 * Enable GPIO 182 = LCD Backlight Power
	 */
	r = gpio_request(LCD_PANEL_BKLIGHT_PWR, "lcd_backlight_pwr");
	if (r) {
		printk(KERN_ERR "failed to get lcd_backlight_pwr\n");
		return;
	}
	gpio_direction_output(LCD_PANEL_BKLIGHT_PWR, 1);
	/*
	 * Enable GPIO 181 = LCD Panel PWM
	 */
	r = gpio_request(LCD_PANEL_PWM, "lcd_pwm");
	if (r) {
		printk(KERN_ERR "failed to get lcd_pwm\n");
		goto err_1;
	}
	gpio_direction_output(LCD_PANEL_PWM, 1);
	/*
	 * Enable GPIO 176 = LCD Panel Power enable pin
	 */
	r = gpio_request(LCD_PANEL_PWR, "lcd_panel_pwr");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_pwr\n");
		goto err_2;
	}
	gpio_direction_output(LCD_PANEL_PWR, 0);

	printk(KERN_INFO "Display initialized successfully\n");
	return;

err_2:
	gpio_free(LCD_PANEL_PWM);
err_1:
	gpio_free(LCD_PANEL_BKLIGHT_PWR);
#else
	printk(KERN_INFO "Display option not selected\n");
#endif
}

static int am3517_evm_panel_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}
	gpio_set_value(LCD_PANEL_PWR, 1);
	lcd_enabled = 1;

	return 0;
}

static void am3517_evm_panel_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(LCD_PANEL_PWR, 0);
	lcd_enabled = 0;
}

static struct omap_dss_device am3517_evm_lcd_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "lcd",
	.driver_name		= "sharp_lq_panel",
	.phy.dpi.data_lines 	= 16,
	.platform_enable	= am3517_evm_panel_enable_lcd,
	.platform_disable	= am3517_evm_panel_disable_lcd,
};

static int am3517_evm_panel_enable_tv(struct omap_dss_device *dssdev)
{
	return 0;
}

static void am3517_evm_panel_disable_tv(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_device am3517_evm_tv_device = {
	.type 			= OMAP_DISPLAY_TYPE_VENC,
	.name 			= "tv",
	.driver_name		= "venc",
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
	.platform_enable	= am3517_evm_panel_enable_tv,
	.platform_disable	= am3517_evm_panel_disable_tv,
};

static int am3517_evm_panel_enable_dvi(struct omap_dss_device *dssdev)
{
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}
	dvi_enabled = 1;

	return 0;
}

static void am3517_evm_panel_disable_dvi(struct omap_dss_device *dssdev)
{
	dvi_enabled = 0;
}

static struct omap_dss_device am3517_evm_dvi_device = {
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.phy.dpi.data_lines	= 24,
	.platform_enable	= am3517_evm_panel_enable_dvi,
	.platform_disable	= am3517_evm_panel_disable_dvi,
};

static struct omap_dss_device *am3517_evm_dss_devices[] = {
	&am3517_evm_lcd_device,
	&am3517_evm_tv_device,
	&am3517_evm_dvi_device,
};

static struct omap_dss_board_info am3517_evm_dss_data = {
	.num_devices	= ARRAY_SIZE(am3517_evm_dss_devices),
	.devices	= am3517_evm_dss_devices,
	.default_device	= &am3517_evm_lcd_device,
};

struct platform_device am3517_evm_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data	= &am3517_evm_dss_data,
	},
};

/*
 * VPFE - Video Decoder interface
 */
#define TVP514X_STD_ALL		(V4L2_STD_NTSC | V4L2_STD_PAL)

/* Inputs available at the TVP5146 */
static struct v4l2_input tvp5146_inputs[] = {
	{
		.index	= 0,
		.name	= "Composite",
		.type	= V4L2_INPUT_TYPE_CAMERA,
		.std	= TVP514X_STD_ALL,
	},
	{
		.index	= 1,
		.name	= "S-Video",
		.type	= V4L2_INPUT_TYPE_CAMERA,
		.std	= TVP514X_STD_ALL,
	},
};

static struct tvp514x_platform_data tvp5146_pdata = {
	.clk_polarity	= 0,
	.hs_polarity	= 1,
	.vs_polarity	= 1
};

static struct vpfe_route tvp5146_routes[] = {
	{
		.input	= INPUT_CVBS_VI1A,
		.output	= OUTPUT_10BIT_422_EMBEDDED_SYNC,
	},
	{
		.input	= INPUT_SVIDEO_VI2C_VI1C,
		.output	= OUTPUT_10BIT_422_EMBEDDED_SYNC,
	},
};

static struct vpfe_subdev_info vpfe_sub_devs[] = {
	{
		.name		= "tvp5146",
		.grp_id		= 0,
		.num_inputs	= ARRAY_SIZE(tvp5146_inputs),
		.inputs		= tvp5146_inputs,
		.routes		= tvp5146_routes,
		.can_route	= 1,
		.ccdc_if_params	= {
			.if_type = VPFE_BT656_10BIT,
			.hdpol	= VPFE_PINPOL_POSITIVE,
			.vdpol	= VPFE_PINPOL_POSITIVE,
		},
		.board_info	= {
			I2C_BOARD_INFO("tvp5146", 0x5C),
			.platform_data = &tvp5146_pdata,
		},
	},
};

static void am3517_evm_clear_vpfe_intr(int vdint)
{
	unsigned int vpfe_int_clr;

	vpfe_int_clr = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);

	switch (vdint) {
	/* VD0 interrrupt */
	case INT_35XX_CCDC_VD0_IRQ:
		vpfe_int_clr &= ~AM35XX_VPFE_CCDC_VD0_INT_CLR;
		vpfe_int_clr |= AM35XX_VPFE_CCDC_VD0_INT_CLR;
		break;
	/* VD1 interrrupt */
	case INT_35XX_CCDC_VD1_IRQ:
		vpfe_int_clr &= ~AM35XX_VPFE_CCDC_VD1_INT_CLR;
		vpfe_int_clr |= AM35XX_VPFE_CCDC_VD1_INT_CLR;
		break;
	/* VD2 interrrupt */
	case INT_35XX_CCDC_VD2_IRQ:
		vpfe_int_clr &= ~AM35XX_VPFE_CCDC_VD2_INT_CLR;
		vpfe_int_clr |= AM35XX_VPFE_CCDC_VD2_INT_CLR;
		break;
	/* Clear all interrrupts */
	default:
		vpfe_int_clr &= ~(AM35XX_VPFE_CCDC_VD0_INT_CLR |
				AM35XX_VPFE_CCDC_VD1_INT_CLR |
				AM35XX_VPFE_CCDC_VD2_INT_CLR);
		vpfe_int_clr |= (AM35XX_VPFE_CCDC_VD0_INT_CLR |
				AM35XX_VPFE_CCDC_VD1_INT_CLR |
				AM35XX_VPFE_CCDC_VD2_INT_CLR);
		break;
	}
	omap_ctrl_writel(vpfe_int_clr, AM35XX_CONTROL_LVL_INTR_CLEAR);
	vpfe_int_clr = omap_ctrl_readl(AM35XX_CONTROL_LVL_INTR_CLEAR);
}

static struct vpfe_config vpfe_cfg = {
	.num_subdevs	= ARRAY_SIZE(vpfe_sub_devs),
	.i2c_adapter_id	= 3,
	.sub_devs	= vpfe_sub_devs,
	.clr_intr	= am3517_evm_clear_vpfe_intr,
	.card_name	= "DM6446 EVM",
	.ccdc		= "DM6446 CCDC",
};

static struct resource vpfe_resources[] = {
	{
		.start	= INT_35XX_CCDC_VD0_IRQ,
		.end	= INT_35XX_CCDC_VD0_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= INT_35XX_CCDC_VD1_IRQ,
		.end	= INT_35XX_CCDC_VD1_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static u64 vpfe_capture_dma_mask = DMA_BIT_MASK(32);
static struct platform_device vpfe_capture_dev = {
	.name		= CAPTURE_DRV_NAME,
	.id		= -1,
	.num_resources	= ARRAY_SIZE(vpfe_resources),
	.resource	= vpfe_resources,
	.dev = {
		.dma_mask		= &vpfe_capture_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
		.platform_data		= &vpfe_cfg,
	},
};

static struct resource dm644x_ccdc_resource[] = {
	/* CCDC Base address */
	{
		.start	= AM35XX_IPSS_VPFE_BASE,
		.end	= AM35XX_IPSS_VPFE_BASE + 0xffff,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device dm644x_ccdc_dev = {
	.name		= "dm644x_ccdc",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(dm644x_ccdc_resource),
	.resource	= dm644x_ccdc_resource,
	.dev = {
		.dma_mask		= &vpfe_capture_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};

/* TPS65023 specific initialization */
/* VDCDC1 -> VDD_CORE */
static struct regulator_consumer_supply am3517_evm_vdcdc1_supplies[] = {
	{
		.supply = "vdd_core",
	},
};

/* VDCDC2 -> VDDSHV */
static struct regulator_consumer_supply am3517_evm_vdcdc2_supplies[] = {
	{
		.supply = "vddshv",
	},
};

/* VDCDC2 |-> VDDS
	   |-> VDDS_SRAM_CORE_BG
	   |-> VDDS_SRAM_MPU */
static struct regulator_consumer_supply am3517_evm_vdcdc3_supplies[] = {
	{
		.supply = "vdds",
	},
	{
		.supply = "vdds_sram_core_bg",
	},
	{
		.supply = "vdds_sram_mpu",
	},
};

/* LDO1 |-> VDDA1P8V_USBPHY
	 |-> VDDA_DAC */
static struct regulator_consumer_supply am3517_evm_ldo1_supplies[] = {
	{
		.supply = "vdda1p8v_usbphy",
	},
	{
		.supply = "vdda_dac",
	},
};

/* LDO2 -> VDDA3P3V_USBPHY */
static struct regulator_consumer_supply am3517_evm_ldo2_supplies[] = {
	{
		.supply = "vdda3p3v_usbphy",
	},
};

static struct regulator_init_data am3517_evm_regulator_data[] = {
	/* DCDC1 */
	{
		.constraints = {
			.min_uV = 1200000,
			.max_uV = 1200000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_vdcdc1_supplies),
		.consumer_supplies = am3517_evm_vdcdc1_supplies,
	},
	/* DCDC2 */
	{
		.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_vdcdc2_supplies),
		.consumer_supplies = am3517_evm_vdcdc2_supplies,
	},
	/* DCDC3 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = true,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_vdcdc3_supplies),
		.consumer_supplies = am3517_evm_vdcdc3_supplies,
	},
	/* LDO1 */
	{
		.constraints = {
			.min_uV = 1800000,
			.max_uV = 1800000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = false,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_ldo1_supplies),
		.consumer_supplies = am3517_evm_ldo1_supplies,
	},
	/* LDO2 */
	{
		.constraints = {
			.min_uV = 3300000,
			.max_uV = 3300000,
			.valid_modes_mask = REGULATOR_MODE_NORMAL,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS,
			.always_on = false,
			.apply_uV = false,
		},
		.num_consumer_supplies = ARRAY_SIZE(am3517_evm_ldo2_supplies),
		.consumer_supplies = am3517_evm_ldo2_supplies,
	},
};

/*
 * TSC 2004 Support
 */
#define	GPIO_TSC2004_IRQ	65

static int tsc2004_init_irq(void)
{
	int ret = 0;

	ret = gpio_request(GPIO_TSC2004_IRQ, "tsc2004-irq");
	if (ret < 0) {
		printk(KERN_WARNING "failed to request GPIO#%d: %d\n",
				GPIO_TSC2004_IRQ, ret);
		return ret;
	}

	if (gpio_direction_input(GPIO_TSC2004_IRQ)) {
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
				"input\n", GPIO_TSC2004_IRQ);
		return -ENXIO;
	}

	omap_set_gpio_debounce(GPIO_TSC2004_IRQ, 1);
	omap_set_gpio_debounce_time(GPIO_TSC2004_IRQ, 0xa);
	return ret;
}

static void tsc2004_exit_irq(void)
{
	gpio_free(GPIO_TSC2004_IRQ);
}

static int tsc2004_get_irq_level(void)
{
	return gpio_get_value(GPIO_TSC2004_IRQ) ? 0 : 1;
}

struct tsc2004_platform_data am3517evm_tsc2004data = {
	.model = 2004,
	.x_plate_ohms = 180,
	.get_pendown_state = tsc2004_get_irq_level,
	.init_platform_hw = tsc2004_init_irq,
	.exit_platform_hw = tsc2004_exit_irq,
};

/*
 * RTC - S35390A
 */
#define	GPIO_RTCS35390A_IRQ	55

static struct i2c_board_info __initdata am3517evm_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("tsc2004", 0x4B),
		.type		= "tsc2004",
		.platform_data	= &am3517evm_tsc2004data,
	},
	{
		I2C_BOARD_INFO("s35390a", 0x30),
		.type		= "s35390a",
	},
	{
		I2C_BOARD_INFO("tps65023", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &am3517_evm_regulator_data[0],
	},
};

/* TCA6416 I2C Expander */
static struct pca953x_platform_data am3517evm_gpio_expander_info = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
};

static struct i2c_board_info __initdata am3517evm_i2c2_boardinfo[] = {
	{
		I2C_BOARD_INFO("tlv320aic23", 0x1A),
	},
	{
		I2C_BOARD_INFO("tca6416", 0x21),
		.platform_data = &am3517evm_gpio_expander_info,
	},
};

/*Keypad Initialization */
#define KEYPAD_IRQ		160
#define KEYPAD_PIN_MASK		0xFFC0

#define KEYPAD_BUTTON(ev_type, ev_code, act_low, descr) \
{                                                               \
        .type           = ev_type,                              \
        .code           = ev_code,                              \
        .active_low     = act_low,                              \
        .desc           = "btn " descr,                         \
}

#define KEYPAD_BUTTON_LOW(event_code, description)      \
        KEYPAD_BUTTON(EV_KEY, event_code, 1, description)

static struct gpio_keys_button am3517_gpio_keys[] = {
	KEYPAD_BUTTON_LOW( KEY_DOWN,	"down"),
	KEYPAD_BUTTON_LOW( KEY_UP,	"up"),
	KEYPAD_BUTTON_LOW( KEY_MENU,	"menu"),
	KEYPAD_BUTTON_LOW( KEY_MODE,	"mode"),
	KEYPAD_BUTTON_LOW( KEY_LEFTSHIFT,"shift"),
	KEYPAD_BUTTON_LOW( KEY_REWIND,	"rewind"),
	KEYPAD_BUTTON_LOW( KEY_FORWARD,	"forward"),
	KEYPAD_BUTTON_LOW( KEY_STOP,	"stop"),
	KEYPAD_BUTTON_LOW( KEY_PLAY,	"play"),
	KEYPAD_BUTTON_LOW( KEY_RECORD,	"rec"),
};


static struct tca6416_keys_platform_data am3517evm_tca6416_keys_info = {
	.buttons	= am3517_gpio_keys,
	.nbuttons	= ARRAY_SIZE(am3517_gpio_keys),
	.rep 		= 0,
	.use_polling	= 1,
	.pinmask	= KEYPAD_PIN_MASK,
};
static struct pca953x_platform_data am3517evm_ui_gpio_expander_info_2 = {
	.gpio_base	= OMAP_MAX_GPIO_LINES + 16,
};
static struct i2c_board_info __initdata am3517evm_ui_tca6516_info[] = {
	{
		I2C_BOARD_INFO("tca6416-keys", 0x20),
		.platform_data = &am3517evm_tca6416_keys_info,
	},
	{
		I2C_BOARD_INFO("tca6416", 0x21),
		.platform_data = &am3517evm_ui_gpio_expander_info_2,
	},
};

static int tca6416_keypad_init_irq(void)
{
	int ret = 0;

	ret = gpio_request(KEYPAD_IRQ, "tca6416-keypad-irq");
	if (ret < 0) {
		printk(KERN_WARNING "failed to request GPIO#%d: %d\n",
				KEYPAD_IRQ, ret);
		return ret;
	}

	if (gpio_direction_input(KEYPAD_IRQ)) {
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
				"input\n", KEYPAD_IRQ);
		return -ENXIO;
	}


	return ret;
}
static int __init am3517_evm_i2c_init(void)
{
	omap_register_i2c_bus(1, 400, NULL, 0);
	omap_register_i2c_bus(2, 400, am3517evm_i2c2_boardinfo,
			ARRAY_SIZE(am3517evm_i2c2_boardinfo));
	omap_register_i2c_bus(3, 400, am3517evm_ui_tca6516_info,
			ARRAY_SIZE(am3517evm_ui_tca6516_info));

	return 0;
}

/*
 * HECC information
 */

#define CAN_STB		214
static void am3517_hecc_plat_init(void)
{
	int r;

        r = gpio_request(CAN_STB, "can_stb");
        if (r) {
                printk(KERN_ERR "failed to get can_stb \n");
		return;
        }

        gpio_direction_output(CAN_STB, 0);
}

static struct resource am3517_hecc_resources[] = {
        {
                .start  = AM35XX_IPSS_HECC_BASE,
                .end    = AM35XX_IPSS_HECC_BASE + 0x3FFF,
                .flags  = IORESOURCE_MEM,
        },
        {
                .start  = INT_35XX_HECC0_IRQ,
                .end    = INT_35XX_HECC0_IRQ,
                .flags  = IORESOURCE_IRQ,
        },
};

static struct platform_device am3517_hecc_device = {
        .name           = "ti_hecc",
        .id             = 1,
        .num_resources  = ARRAY_SIZE(am3517_hecc_resources),
        .resource       = am3517_hecc_resources,
};

static struct ti_hecc_platform_data am3517_evm_hecc_pdata = {
        .scc_hecc_offset        = AM35XX_HECC_SCC_HECC_OFFSET,
        .scc_ram_offset         = AM35XX_HECC_SCC_RAM_OFFSET,
        .hecc_ram_offset        = AM35XX_HECC_RAM_OFFSET,
        .mbx_offset            = AM35XX_HECC_MBOX_OFFSET,
        .int_line               = AM35XX_HECC_INT_LINE,
        .version                = AM35XX_HECC_VERSION,
	.platform_init		= am3517_hecc_plat_init,
};

static void am3517_evm_hecc_init(struct ti_hecc_platform_data *pdata)
{
        am3517_hecc_device.dev.platform_data = pdata;
        platform_device_register(&am3517_hecc_device);
}


/*
 * Board initialization
 */
static struct omap_board_config_kernel am3517_evm_config[] __initdata = {
};

static struct platform_device *am3517_evm_devices[] __initdata = {
	&dm644x_ccdc_dev,
	&vpfe_capture_dev,
	&am3517_evm_dss_device,
};

static void __init am3517_evm_init_irq(void)
{
	omap_board_config = am3517_evm_config;
	omap_board_config_size = ARRAY_SIZE(am3517_evm_config);

	omap2_init_common_hw(NULL, NULL, NULL, NULL, NULL);
	omap_init_irq();
	omap_gpio_init();
}

static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
#if defined(CONFIG_PANEL_SHARP_LQ043T1DG01) || \
		defined(CONFIG_PANEL_SHARP_LQ043T1DG01_MODULE)
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
#else
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
#endif
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = 57,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	/* USB OTG DRVVBUS offset = 0x212 */
	OMAP3_MUX(CHASSIS_DMAREQ3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP_CLKS, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(GPMC_NCS4, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),
	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define board_mux	NULL
#endif

static struct am3517_hsmmc_info mmc[] = {
	{
		.mmc            = 1,
		.wires          = 4,
		/*TODO: Need to change*/
		.gpio_cd        = 127,
		.gpio_wp        = 126,
	},
	{
		.mmc            = 2,
		.wires          = 4,
		/*TODO: Need to change*/
		.gpio_cd        = 128,
		.gpio_wp        = 129,
	},
	{}      /* Terminator */
};

static void __init am3517_evm_init(void)
{

	/* Init TCA6416 keypad */
	tca6416_keypad_init_irq();

	am3517_evm_i2c_init();

	omap3_mux_init(board_mux, OMAP_PACKAGE_CBB);
	platform_add_devices(am3517_evm_devices,
				ARRAY_SIZE(am3517_evm_devices));

	omap_serial_init();
	am3517evm_flash_init();
	usb_musb_init();
	/* Configure GPIO for EHCI port */
	omap_mux_init_gpio(57, OMAP_PIN_OUTPUT);
	usb_ehci_init(&ehci_pdata);

	/* TSC 2004 */
	omap_mux_init_gpio(65, OMAP_PIN_INPUT_PULLUP);
	am3517evm_i2c1_boardinfo[0].irq = gpio_to_irq(GPIO_TSC2004_IRQ);

	/* RTC - S35390A */
	omap_mux_init_gpio(55, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(GPIO_RTCS35390A_IRQ, "rtcs35390a-irq") < 0)
		printk(KERN_WARNING "failed to request GPIO#%d\n",
				GPIO_RTCS35390A_IRQ);
	if (gpio_direction_input(GPIO_RTCS35390A_IRQ))
		printk(KERN_WARNING "GPIO#%d cannot be configured as "
				"input\n", GPIO_RTCS35390A_IRQ);
	am3517evm_i2c1_boardinfo[1].irq = gpio_to_irq(GPIO_RTCS35390A_IRQ);

	i2c_register_board_info(1, am3517evm_i2c1_boardinfo,
				ARRAY_SIZE(am3517evm_i2c1_boardinfo));

	clk_add_alias("master", "dm644x_ccdc", "master",
			&vpfe_capture_dev.dev);
	clk_add_alias("slave", "dm644x_ccdc", "slave",
			&vpfe_capture_dev.dev);
	/* DSS */
	am3517_evm_display_init();

	/*Ethernet*/
	am3517_evm_ethernet_init(&am3517_evm_emac_pdata);
	am3517_evm_hecc_init(&am3517_evm_hecc_pdata);

	/* MMC init function */
	am3517_mmc_init(mmc);

}

static void __init am3517_evm_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP3517EVM, "OMAP3517/AM3517 EVM")
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xd8000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= am3517_evm_map_io,
	.init_irq	= am3517_evm_init_irq,
	.init_machine	= am3517_evm_init,
	.timer		= &omap_timer,
MACHINE_END
