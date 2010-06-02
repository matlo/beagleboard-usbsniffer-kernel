/*
 * ispccdc.h
 *
 * Driver header file for CCDC module in TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contributors:
 *	Senthilvadivu Guruswamy <svadivu@ti.com>
 *	Pallavi Kulkarni <p-kulkarni@ti.com>
 *	Sergio Aguirre <saaguirre@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_ISP_CCDC_H
#define OMAP_ISP_CCDC_H

#include <linux/workqueue.h>
#include <plat/isp_user.h>
#include "ispvideo.h"

/* Enumeration constants for the sync interface parameters */
enum inpmode {
	RAW,
	YUV16,
	YUV8
};
enum datasize {
	DAT8,
	DAT10,
	DAT11,
	DAT12
};

enum ccdc_input_entity {
	CCDC_INPUT_NONE,
	CCDC_INPUT_PARALLEL,
	CCDC_INPUT_CSI2A,
	CCDC_INPUT_CCP2B,
	CCDC_INPUT_CSI2C
};

#define CCDC_OUTPUT_MEMORY	(1 << 0)
#define CCDC_OUTPUT_PREVIEW	(1 << 1)
#define CCDC_OUTPUT_RESIZER	(1 << 2)

#define	OMAP3ISP_CCDC_NEVENTS	16

/**
 * struct ispccdc_syncif - Structure for Sync Interface between sensor and CCDC
 * @ccdc_mastermode: Master mode. 1 - Master, 0 - Slave.
 * @fldstat: Field state. 0 - Odd Field, 1 - Even Field.
 * @datsz: Data size.
 * @fldmode: 0 - Progressive, 1 - Interlaced.
 * @datapol: 0 - Positive, 1 - Negative.
 * @fldpol: 0 - Positive, 1 - Negative.
 * @hdpol: 0 - Positive, 1 - Negative.
 * @vdpol: 0 - Positive, 1 - Negative.
 * @fldout: 0 - Input, 1 - Output.
 * @hs_width: Width of the Horizontal Sync pulse, used for HS/VS Output.
 * @vs_width: Width of the Vertical Sync pulse, used for HS/VS Output.
 * @ppln: Number of pixels per line, used for HS/VS Output.
 * @hlprf: Number of half lines per frame, used for HS/VS Output.
 * @bt_r656_en: 1 - Enable ITU-R BT656 mode, 0 - Sync mode.
 */
struct ispccdc_syncif {
	u8 ccdc_mastermode;
	u8 fldstat;
	enum datasize datsz;
	u8 fldmode;
	u8 datapol;
	u8 fldpol;
	u8 hdpol;
	u8 vdpol;
	u8 fldout;
	u8 hs_width;
	u8 vs_width;
	u8 ppln;
	u8 hlprf;
	u8 bt_r656_en;
};

/*
 * struct ispccdc_vp - Structure for Video Port parameters
 * @bitshift_sel: Video port input select. 3 - bits 12-3, 4 - bits 11-2,
 *                5 - bits 10-1, 6 - bits 9-0.
 * @pixelclk: Input pixel clock in Hz
 */
struct ispccdc_vp {
	enum vpin bitshift_sel;
	unsigned int pixelclk;
};

/*
 * ispccdc_lsc - CCDC LSC parameters
 * @update_config: Set when user changes config
 * @request_enable: Whether LSC is requested to be enabled
 * @config: LSC config set by user
 * @update_table: Set when user provides a new LSC table to table_new
 * @table_new: LSC table set by user, ISP address
 * @table_inuse: LSC table currently in use, ISP address
 */
struct ispccdc_lsc {
	unsigned int update_config:1,
		     request_enable:1,
		     update_table:1;
	struct ispccdc_lsc_config config;
	struct work_struct table_work;
	u32 table_old;
	u32 table_new;
	u32 table_inuse;
};

/* Sink and source CCDC pads */
#define CCDC_PAD_SINK			0
#define CCDC_PAD_SOURCE_OF		1
#define CCDC_PAD_SOURCE_VP		2
#define CCDC_PADS_NUM			3

/**
 * struct isp_ccdc_device - Structure for the CCDC module to store its own
 *			    information
 * @subdev: V4L2 subdevice
 * @pads: Sink and source media entity pads
 * @formats: Probe and active video formats
 * @video_out: Output video node
 * @error: A hardware error occured during capture
 * @ccdcin_woffset: CCDC input horizontal offset.
 * @ccdcin_hoffset: CCDC input vertical offset.
 * @crop_w: Crop width.
 * @crop_h: Crop weight.
 * @syncif_ipmod: Image
 * @obclamp_en: Data input format.
 * @fpc_table_add_m: ISP MMU mapped address of the current used FPC table.
 * @fpc_table_add: Virtual address of the current used FPC table.
 * @shadow_update: non-zero when user is updating CCDC configuration
 * @enabled: Whether the CCDC is enabled
 * @underrun: A buffer underrun occured and a new buffer has been queued
 * @lock: serializes shadow_update with interrupt handler
 */
struct isp_ccdc_device {
	struct v4l2_subdev subdev;
	struct media_entity_pad pads[CCDC_PADS_NUM];
	struct v4l2_mbus_framefmt formats[CCDC_PADS_NUM][2];

	enum ccdc_input_entity input;
	unsigned int output;
	struct isp_video video_out;
	struct isp_video *video;
	unsigned int error;

	u32 ccdcin_woffset;
	u32 ccdcin_hoffset;
	u32 crop_w;
	u32 crop_h;
	u8 syncif_ipmod;
	u8 obclamp_en;
	unsigned long fpc_table_add_m;
	u32 *fpc_table_add;
	struct ispccdc_lsc lsc;
	struct ispccdc_bclamp blkcfg;
	struct ispccdc_syncif syncif;
	struct ispccdc_vp vpcfg;

	unsigned int shadow_update:1,
		     underrun:1;
	enum isp_pipeline_stream_state state;
	spinlock_t lock;
	wait_queue_head_t wait;
	unsigned int stopping;
};

struct isp_device;

int isp_ccdc_init(struct isp_device *isp);
void isp_ccdc_cleanup(struct isp_device *isp);
int isp_ccdc_register_entities(struct isp_ccdc_device *ccdc,
	struct v4l2_device *vdev);
void isp_ccdc_unregister_entities(struct isp_ccdc_device *ccdc);

void ispccdc_lsc_error_handler(struct isp_ccdc_device *isp_ccdc);
int ispccdc_busy(struct isp_ccdc_device *isp_ccdc);
int ispccdc_isr(struct isp_ccdc_device *isp_ccdc);
void ispccdc_hs_vs_isr(struct isp_ccdc_device *isp_ccdc);
void ispccdc_save_context(struct isp_device *isp);
void ispccdc_restore_context(struct isp_device *isp);

#endif		/* OMAP_ISP_CCDC_H */
