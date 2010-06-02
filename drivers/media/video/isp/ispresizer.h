/*
 * ispresizer.h
 *
 * Driver header file for Resizer module in TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 *
 * Contributors:
 * 	Sameer Venkatraman <sameerv@ti.com>
 * 	Mohit Jalori
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

#ifndef OMAP_ISP_RESIZER_H
#define OMAP_ISP_RESIZER_H

/*
 * Constants for filter coefficents count
 */
#define COEFF_CNT		32

enum resizer_input {
	RSZ_OTFLY_YUV,
	RSZ_MEM_YUV,
	RSZ_MEM_COL8
};

/**
 * struct isprsz_coef - Structure for resizer filter coeffcients.
 * @h_filter_coef_4tap: Horizontal filter coefficients for 8-phase/4-tap
 *			mode (.5x-4x)
 * @v_filter_coef_4tap: Vertical filter coefficients for 8-phase/4-tap
 *			mode (.5x-4x)
 * @h_filter_coef_7tap: Horizontal filter coefficients for 4-phase/7-tap
 *			mode (.25x-.5x)
 * @v_filter_coef_7tap: Vertical filter coefficients for 4-phase/7-tap
 *			mode (.25x-.5x)
 */
struct isprsz_coef {
	u16 h_filter_coef_4tap[32];
	u16 v_filter_coef_4tap[32];
	/* Every 8th value is a dummy value in the following arrays: */
	u16 h_filter_coef_7tap[32];
	u16 v_filter_coef_7tap[32];
};

/* Chrominance horizontal algorithm */
enum resizer_chroma_algo {
	RSZ_THE_SAME = 0,	/* Chrominance the same as Luminance */
	RSZ_BILINEAR = 1,	/* Chrominance uses bilinear interpolation */
};

/* Resizer input type select */
enum resizer_colors_type {
	RSZ_YUV422 = 0,		/* YUV422 color is interleaved */
	RSZ_COLOR8 = 1,		/* Color separate data on 8 bits */
};

/*
 * Structure for horizontal and vertical resizing value
 */
struct resizer_ratio {
	u32 horz;
	u32 vert;
};

/*
 * Structure for luminance enhancer parameters.
 */
struct resizer_luma_yenh {
	u8 algo;		/* algorithm select. */
	u8 gain;		/* maximum gain. */
	u8 slope;		/* slope. */
	u8 core;		/* core offset. */
};

enum resizer_input_entity {
	RESIZER_INPUT_NONE,
	RESIZER_INPUT_VP,	/* input video port - prev or ccdc */
	RESIZER_INPUT_MEMORY,
};

/* Sink and source resizer pads */
#define RESZ_PAD_SINK			0
#define RESZ_PAD_SOURCE			1
#define RESZ_PADS_NUM			2

/**
 * struct isp_res_device - Structure for the resizer module to store its
 * information.
 */
struct isp_res_device {
	struct v4l2_subdev subdev;
	struct media_entity_pad pads[RESZ_PADS_NUM];
	struct v4l2_mbus_framefmt formats[RESZ_PADS_NUM][2];

	enum resizer_input_entity input;
	struct isp_video video_in;
	struct isp_video video_out;
	struct isp_video *video;
	unsigned int error;

	u32 addr_base;   /* stored source buffer address in memory mode */
	u32 crop_offset; /* additional offset for crop in memory mode */
	struct resizer_ratio ratio;
	int pm_state;
	unsigned int applycrop:1,
		     underrun:1;
	enum isp_pipeline_stream_state state;
	struct v4l2_crop crop; /* current crop requested by user */
};

struct isp_device;

int isp_resizer_init(struct isp_device *isp);
void isp_resizer_cleanup(struct isp_device *isp);

int isp_resizer_register_entities(struct isp_res_device *res,
				  struct v4l2_device *vdev);
void isp_resizer_unregister_entities(struct isp_res_device *res);
void ispresizer_isr_frame_sync(struct isp_res_device *res);
void ispresizer_isr(struct isp_res_device *isp_res);

void ispresizer_suspend(struct isp_res_device *isp_res);

void ispresizer_resume(struct isp_res_device *isp_res);

int ispresizer_busy(struct isp_res_device *isp_res);

void ispresizer_save_context(struct isp_device *isp);

void ispresizer_restore_context(struct isp_device *isp);

#endif		/* OMAP_ISP_RESIZER_H */
