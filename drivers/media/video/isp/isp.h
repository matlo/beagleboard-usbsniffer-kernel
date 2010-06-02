/*
 * isp.h
 *
 * Top level public header file for ISP Control module in
 * TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments.
 * Copyright (C) 2009 Nokia.
 *
 * Contributors:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Mohit Jalori
 * 	Sergio Aguirre <saaguirre@ti.com>
 * 	Sakari Ailus <sakari.ailus@nokia.com>
 * 	Tuukka Toivonen <tuukka.o.toivonen@nokia.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_ISP_TOP_H
#define OMAP_ISP_TOP_H

#include <media/v4l2-device.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <plat/iommu.h>
#include <plat/iovmm.h>

struct isp_interface_config;

#include "ispstat.h"
#include "ispccdc.h"
#include "ispreg.h"
#include "ispresizer.h"
#include "isppreview.h"
#include "ispcsiphy.h"
#include "ispcsi2.h"
#include "ispccp2.h"

#define IOMMU_FLAG (IOVMF_ENDIAN_LITTLE | IOVMF_ELSZ_8)

#define OMAP_ISP_CCDC		(1 << 0)
#define OMAP_ISP_PREVIEW	(1 << 1)
#define OMAP_ISP_RESIZER	(1 << 2)
#define OMAP_ISP_AEWB		(1 << 3)
#define OMAP_ISP_AF		(1 << 4)
#define OMAP_ISP_HIST		(1 << 5)

#define ISP_TOK_TERM		0xFFFFFFFF	/*
						 * terminating token for ISP
						 * modules reg list
						 */
#define ISP_BYTES_PER_PIXEL		2

#define to_isp_device(ptr_module)				\
	container_of(ptr_module, struct isp_device, isp_##ptr_module)
#define to_device(ptr_module)						\
	(to_isp_device(ptr_module)->dev)

enum isp_mem_resources {
	OMAP3_ISP_IOMEM_MAIN,
	OMAP3_ISP_IOMEM_CBUFF,
	OMAP3_ISP_IOMEM_CCP2,
	OMAP3_ISP_IOMEM_CCDC,
	OMAP3_ISP_IOMEM_HIST,
	OMAP3_ISP_IOMEM_H3A,
	OMAP3_ISP_IOMEM_PREV,
	OMAP3_ISP_IOMEM_RESZ,
	OMAP3_ISP_IOMEM_SBL,
	OMAP3_ISP_IOMEM_CSI2A_REGS1,
	OMAP3_ISP_IOMEM_CSIPHY2,
	OMAP3_ISP_IOMEM_CSI2A_REGS2,
	OMAP3_ISP_IOMEM_CSI2C_REGS1,
	OMAP3_ISP_IOMEM_CSIPHY1,
	OMAP3_ISP_IOMEM_CSI2C_REGS2,
	OMAP3_ISP_IOMEM_LAST
};

enum isp_sbl_resource {
	OMAP3_ISP_SBL_CSI1_READ		= 0x1,
	OMAP3_ISP_SBL_CSI1_WRITE	= 0x2,
	OMAP3_ISP_SBL_CSI2A_WRITE	= 0x4,
	OMAP3_ISP_SBL_CSI2C_WRITE	= 0x8,
	OMAP3_ISP_SBL_CCDC_LSC_READ	= 0x10,
	OMAP3_ISP_SBL_CCDC_WRITE	= 0x20,
	OMAP3_ISP_SBL_PREVIEW_READ	= 0x40,
	OMAP3_ISP_SBL_PREVIEW_WRITE	= 0x80,
	OMAP3_ISP_SBL_RESIZER_READ	= 0x100,
	OMAP3_ISP_SBL_RESIZER_WRITE	= 0x200,
};

enum isp_interface_type {
	ISP_INTERFACE_PARALLEL,
	ISP_INTERFACE_CSI2A_PHY2,
	ISP_INTERFACE_CCP2B_PHY1,
	ISP_INTERFACE_CCP2B_PHY2,
	ISP_INTERFACE_CSI2C_PHY1,
};

#define ISP_REVISION_1_0		0x10
#define ISP_REVISION_2_0		0x20
#define ISP_REVISION_15_0		0xF0

/**
 * struct isp_res_mapping - Map ISP io resources to ISP revision.
 * @isp_rev: ISP_REVISION_x_x
 * @map: bitmap for enum isp_mem_resources
 */
struct isp_res_mapping {
	u32 isp_rev;
	u32 map;
};

/**
 * struct isp_reg - Structure for ISP register values.
 * @reg: 32-bit Register address.
 * @val: 32-bit Register value.
 */
struct isp_reg {
	enum isp_mem_resources mmio_range;
	u32 reg;
	u32 val;
};

/**
 * struct isp_parallel_platform_data - Parallel interface platform data
 * @data_lane_shift: Data lane shifter
 *		0 - CAMEXT[13:0] -> CAM[13:0]
 *		1 - CAMEXT[13:2] -> CAM[11:0]
 *		2 - CAMEXT[13:4] -> CAM[9:0]
 *		3 - CAMEXT[13:6] -> CAM[7:0]
 * @clk_pol: Pixel clock polarity
 *		0 - Non Inverted, 1 - Inverted
 * @bridge: CCDC Bridge input control
 *		ISPCTRL_PAR_BRIDGE_DISABLE - Disable
 *		ISPCTRL_PAR_BRIDGE_LENDIAN - Little endian
 *		ISPCTRL_PAR_BRIDGE_BENDIAN - Big endian
 */
struct isp_parallel_platform_data {
	unsigned int data_lane_shift;
	unsigned int clk_pol:1;
	unsigned int bridge:2;
};

/**
 * struct isp_ccp2_platform_data - CCP2 interface platform data
 * @data_lane_shift: Data lane shifter
 *		0 - CAMEXT[13:0] -> CAM[13:0]
 *		1 - CAMEXT[13:2] -> CAM[11:0]
 *		2 - CAMEXT[13:4] -> CAM[9:0]
 *		3 - CAMEXT[13:6] -> CAM[7:0]
 * @strobe_clk_pol: Strobe/clock polarity
 *		0 - Non Inverted, 1 - Inverted
 * @crc: Enable the cyclic redundancy check
 * @ccp2_mode: Enable CCP2 compatibility mode
 *		0 - MIPI-CSI1 mode, 1 - CCP2 mode
 * @phy_layer: Physical layer selection
 *		ISPCCP2_CTRL_PHY_SEL_CLOCK - Data/clock physical layer
 * 		ISPCCP2_CTRL_PHY_SEL_STROBE - Data/strobe physical layer
 * @vpclk_div: Video port output clock control
 */
struct isp_ccp2_platform_data {
	unsigned int data_lane_shift;
	unsigned int strobe_clk_pol:1;
	unsigned int crc:1;
	unsigned int ccp2_mode:1;
	unsigned int phy_layer:1;
	unsigned int vpclk_div:2;
};

/**
 * struct isp_csi2_platform_data - CSI2 interface platform data
 * @data_lane_shift: Data lane shifter
 *		0 - CAMEXT[13:0] -> CAM[13:0]
 *		1 - CAMEXT[13:2] -> CAM[11:0]
 *		2 - CAMEXT[13:4] -> CAM[9:0]
 *		3 - CAMEXT[13:6] -> CAM[7:0]
 * @crc: Enable the cyclic redundancy check
 * @vpclk_div: Video port output clock control
 */
struct isp_csi2_platform_data {
	unsigned int data_lane_shift;
	unsigned crc:1;
	unsigned vpclk_div:2;
};

struct isp_platform_data {
	struct isp_parallel_platform_data parallel;
	struct isp_ccp2_platform_data ccp2;
	struct isp_csi2_platform_data csi2a;
	struct isp_csi2_platform_data csi2c;
};

/**
 * struct isp_device - ISP device structure.
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @revision: Stores current ISP module revision.
 * @irq_num: Currently used IRQ number.
 * @mmio_base: Array with kernel base addresses for ioremapped ISP register
 *             regions.
 * @mmio_base_phys: Array with physical L4 bus addresses for ISP register
 *                  regions.
 * @mmio_size: Array with ISP register regions size in bytes.
 * @raw_dmamask: Raw DMA mask
 * @stat_lock: Spinlock for handling statistics
 * @isp_mutex: Mutex for serializing requests to ISP.
 * @has_context: Context has been saved at least once and can be restored.
 * @ref_count: Reference count for handling multiple ISP requests.
 * @cam_ick: Pointer to camera interface clock structure.
 * @cam_mclk: Pointer to camera functional clock structure.
 * @dpll4_m5_ck: Pointer to DPLL4 M5 clock structure.
 * @csi2_fck: Pointer to camera CSI2 complexIO clock structure.
 * @l3_ick: Pointer to OMAP3 L3 bus interface clock.
 * @irq: Currently attached ISP ISR callbacks information structure.
 * @isp_af: Pointer to current settings for ISP AutoFocus SCM.
 * @isp_hist: Pointer to current settings for ISP Histogram SCM.
 * @isp_h3a: Pointer to current settings for ISP Auto Exposure and
 *           White Balance SCM.
 * @isp_res: Pointer to current settings for ISP Resizer.
 * @isp_prev: Pointer to current settings for ISP Preview.
 * @isp_ccdc: Pointer to current settings for ISP CCDC.
 * @iommu: Pointer to requested IOMMU instance for ISP.
 *
 * This structure is used to store the OMAP ISP Information.
 */
struct isp_device {
	struct device *dev;
	u32 revision;

	/*** isp regulators ***/
	struct regulator *vdd_csiphy1;
	struct regulator *vdd_csiphy2;

	/*** platform HW resources ***/
	struct isp_platform_data *pdata;
	unsigned int irq_num;

	void __iomem *mmio_base[OMAP3_ISP_IOMEM_LAST];
	unsigned long mmio_base_phys[OMAP3_ISP_IOMEM_LAST];
	unsigned long mmio_size[OMAP3_ISP_IOMEM_LAST];

	u64 raw_dmamask;

	/* ISP Obj */
	spinlock_t stat_lock;	/* common lock for statistic drivers */
	struct mutex isp_mutex;	/* For handling ref_count field */
	int has_context;
	int ref_count;
	u32 xclk_divisor[2];	/* Two clocks, a and b. */
#define ISP_CLK_CAM_ICK		0
#define ISP_CLK_CAM_MCLK	1
#define ISP_CLK_DPLL4_M5_CK	2
#define ISP_CLK_CSI2_FCK	3
#define ISP_CLK_L3_ICK		4
	struct clk *clock[5];

	/* ISP modules */
	struct ispstat isp_af;
	struct ispstat isp_aewb;
	struct ispstat isp_hist;
	struct isp_res_device isp_res;
	struct isp_prev_device isp_prev;
	struct isp_ccdc_device isp_ccdc;
	struct isp_csi2_device isp_csi2a;
	struct isp_csi2_device isp_csi2c;
	struct isp_ccp2_device isp_ccp2;
	struct isp_csiphy isp_csiphy1;
	struct isp_csiphy isp_csiphy2;

	unsigned int sbl_resources;

	struct iommu *iommu;
};

void isphist_dma_done(struct isp_device *isp);

void isp_flush(struct isp_device *isp);

void isp_stop(struct isp_device *isp, struct isp_video *video);

int isp_pipeline_set_stream(struct isp_device *isp, struct isp_video *node,
			    enum isp_pipeline_stream_state state);
void isp_select_bridge_input(struct isp_device *isp,
			     enum ccdc_input_entity input);

u32 isp_set_xclk(struct isp_device *isp, u32 xclk, u8 xclksel);

void isp_set_pixel_clock(struct isp_device *isp, unsigned int pixelclk);

struct isp_device *isp_get(struct isp_device *isp);
void isp_put(struct isp_device *isp);

int isp_handle_private(struct isp_device *isp, int cmd, void *arg);

void isp_save_context(struct isp_device *isp, struct isp_reg *);

void isp_restore_context(struct isp_device *isp, struct isp_reg *);

void isp_print_status(struct isp_device *isp);

void isp_sbl_enable(struct isp_device *isp, enum isp_sbl_resource res);
void isp_sbl_disable(struct isp_device *isp, enum isp_sbl_resource res);

int omap3isp_register_entities(struct platform_device *pdev,
			       struct v4l2_device *v4l2_dev);
void omap3isp_unregister_entities(struct platform_device *pdev);

/**
 * isp_reg_readl - Read value of an OMAP3 ISP register
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @isp_mmio_range: Range to which the register offset refers to.
 * @reg_offset: Register offset to read from.
 *
 * Returns an unsigned 32 bit value with the required register contents.
 **/
static inline
u32 isp_reg_readl(struct isp_device *isp, enum isp_mem_resources isp_mmio_range,
		  u32 reg_offset)
{
	return __raw_readl(isp->mmio_base[isp_mmio_range] + reg_offset);
}

/**
 * isp_reg_writel - Write value to an OMAP3 ISP register
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @reg_value: 32 bit value to write to the register.
 * @isp_mmio_range: Range to which the register offset refers to.
 * @reg_offset: Register offset to write into.
 **/
static inline
void isp_reg_writel(struct isp_device *isp, u32 reg_value,
		    enum isp_mem_resources isp_mmio_range, u32 reg_offset)
{
	__raw_writel(reg_value, isp->mmio_base[isp_mmio_range] + reg_offset);
}

/**
 * isp_reg_and - Do AND binary operation within an OMAP3 ISP register value
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @mmio_range: Range to which the register offset refers to.
 * @reg: Register offset to work on.
 * @and_bits: 32 bit value which would be 'ANDed' with current register value.
 **/
static inline
void isp_reg_and(struct isp_device *isp, enum isp_mem_resources mmio_range,
		 u32 reg, u32 and_bits)
{
	u32 v = isp_reg_readl(isp, mmio_range, reg);

	isp_reg_writel(isp, v & and_bits, mmio_range, reg);
}

/**
 * isp_reg_or - Do OR binary operation within an OMAP3 ISP register value
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @mmio_range: Range to which the register offset refers to.
 * @reg: Register offset to work on.
 * @or_bits: 32 bit value which would be 'ORed' with current register value.
 **/
static inline
void isp_reg_or(struct isp_device *isp, enum isp_mem_resources mmio_range,
		u32 reg, u32 or_bits)
{
	u32 v = isp_reg_readl(isp, mmio_range, reg);

	isp_reg_writel(isp, v | or_bits, mmio_range, reg);
}

/**
 * isp_reg_and_or - Do AND and OR binary ops within an OMAP3 ISP register value
 * @dev: Device pointer specific to the OMAP3 ISP.
 * @mmio_range: Range to which the register offset refers to.
 * @reg: Register offset to work on.
 * @and_bits: 32 bit value which would be 'ANDed' with current register value.
 * @or_bits: 32 bit value which would be 'ORed' with current register value.
 *
 * The AND operation is done first, and then the OR operation. Mostly useful
 * when clearing a group of bits before setting a value.
 **/
static inline
void isp_reg_and_or(struct isp_device *isp, enum isp_mem_resources mmio_range,
		    u32 reg, u32 and_bits, u32 or_bits)
{
	u32 v = isp_reg_readl(isp, mmio_range, reg);

	isp_reg_writel(isp, (v & and_bits) | or_bits, mmio_range, reg);
}

static inline enum v4l2_buf_type
isp_pad_buffer_type(const struct v4l2_subdev *subdev, int pad)
{
	if (pad >= subdev->entity.num_pads)
		return 0;

	if (subdev->entity.pads[pad].type == MEDIA_PAD_TYPE_INPUT)
		return V4L2_BUF_TYPE_VIDEO_OUTPUT;
	else
		return V4L2_BUF_TYPE_VIDEO_CAPTURE;
}

#endif	/* OMAP_ISP_TOP_H */
