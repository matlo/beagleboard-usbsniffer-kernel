/*
 *ispccp2.h
 *
 * Copyright (C) 2010 Nokia Corporation.
 * Copyright (C) 2010 Texas Instruments.
 *
 * Contributors:
 *      RaniSuneela M <r-m@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef OMAP_ISP_CCP2_API_H
#define OMAP_ISP_CCP2_API_H

#include <linux/videodev2.h>

struct isp_device;
struct isp_csiphy;

/* Sink and source ccp2 pads */
#define CCP2_PAD_SINK			0
#define CCP2_PAD_SOURCE			1
#define CCP2_PADS_NUM			2

/* CCP2 input media entity */
enum ccp2_input_entity {
	CCP2_INPUT_NONE,
	CCP2_INPUT_SENSOR,
	CCP2_INPUT_MEMORY,
};

/* CCP2 output media entity */
enum ccp2_output_entity {
	CCP2_OUTPUT_NONE,
	CCP2_OUTPUT_CCDC,
	CCP2_OUTPUT_MEMORY,
};


/* Logical channel configuration */
struct isp_interface_lcx_config {
	int crc;
	u32 data_start;
	u32 data_size;
	u32 format;
};

/* Memory channel configuration */
struct isp_interface_mem_config {
	u32 dst_port;
	u32 vsize_count;
	u32 hsize_count;
	u32 src_ofst;
	u32 dst_ofst;
	u32 buf_addr;
};

/* CCP2 device */
struct isp_ccp2_device {
	struct v4l2_subdev subdev;
	struct v4l2_mbus_framefmt formats[CCP2_PADS_NUM][2];
	struct media_entity_pad pads[CCP2_PADS_NUM];
	enum ccp2_input_entity input;
	enum ccp2_output_entity output;
	struct isp_interface_lcx_config if_cfg;
	struct isp_interface_mem_config mem_cfg;
	struct isp_video video_in;
	struct isp_csiphy *phy;
	unsigned int error;
	enum isp_pipeline_stream_state state;
};

/* Function declarations */
void ispccp2_save_context(struct isp_device *isp);
void ispccp2_restore_context(struct isp_device *isp);
int isp_ccp2_init(struct isp_device *isp);
void isp_ccp2_cleanup(struct isp_device *isp);
int isp_ccp2_register_entities(struct isp_ccp2_device *ccp2,
			struct v4l2_device *vdev);
void isp_ccp2_unregister_entities(struct isp_ccp2_device *ccp2);
int ispccp2_isr(struct isp_device *isp);

#endif
