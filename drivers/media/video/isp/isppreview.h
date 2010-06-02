/*
 * isppreview.h
 *
 * Driver header file for Preview module in TI's OMAP3 Camera ISP
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

#ifndef OMAP_ISP_PREVIEW_H
#define OMAP_ISP_PREVIEW_H

#include <plat/isp_user.h>
#include "ispvideo.h"
/* Isp query control structure */

#define ISPPRV_BRIGHT_STEP		0x1
#define ISPPRV_BRIGHT_DEF		0x1
#define ISPPRV_BRIGHT_LOW		0x0
#define ISPPRV_BRIGHT_HIGH		0xFF
#define ISPPRV_BRIGHT_UNITS		0x1

#define ISPPRV_CONTRAST_STEP		0x1
#define ISPPRV_CONTRAST_DEF		0x10
#define ISPPRV_CONTRAST_LOW		0x0
#define ISPPRV_CONTRAST_HIGH		0xFF
#define ISPPRV_CONTRAST_UNITS		0x1

#define NO_AVE				0x0
#define AVE_2_PIX			0x1
#define AVE_4_PIX			0x2
#define AVE_8_PIX			0x3

/* Features list */
#define PREV_LUMA_ENHANCE		ISP_PREV_LUMAENH
#define PREV_INVERSE_ALAW 		ISP_PREV_INVALAW
#define PREV_HORZ_MEDIAN_FILTER		ISP_PREV_HRZ_MED
#define PREV_CFA			ISP_PREV_CFA
#define PREV_CHROMA_SUPPRESS		ISP_PREV_CHROMA_SUPP
#define PREV_WB				ISP_PREV_WB
#define PREV_BLKADJ			ISP_PREV_BLKADJ
#define PREV_RGB2RGB			ISP_PREV_RGB2RGB
#define PREV_COLOR_CONV			ISP_PREV_COLOR_CONV
#define PREV_YCLIMITS			ISP_PREV_YC_LIMIT
#define PREV_DEFECT_COR			ISP_PREV_DEFECT_COR
#define PREV_GAMMA_BYPASS		ISP_PREV_GAMMABYPASS
#define PREV_DARK_FRAME_CAPTURE		ISP_PREV_DRK_FRM_CAPTURE
#define PREV_DARK_FRAME_SUBTRACT	ISP_PREV_DRK_FRM_SUBTRACT
#define PREV_LENS_SHADING		ISP_PREV_LENS_SHADING
#define PREV_NOISE_FILTER 		ISP_PREV_NF
#define PREV_GAMMA			ISP_PREV_GAMMA

#define PREV_CONTRAST			(1 << 17)
#define PREV_BRIGHTNESS			(1 << 18)
#define PREV_AVERAGER			(1 << 19)
#define PREV_FEATURES_END		(1 << 20)

enum preview_input_entity {
	PREVIEW_INPUT_NONE,
	PREVIEW_INPUT_CCDC,
	PREVIEW_INPUT_MEMORY,
};

#define PREVIEW_OUTPUT_RESIZER		(1 << 1)
#define PREVIEW_OUTPUT_MEMORY		(1 << 2)

/*
 * Configure byte layout of YUV image
 */
enum preview_ycpos_mode {
	YCPOS_YCrYCb = 0,
	YCPOS_YCbYCr = 1,
	YCPOS_CbYCrY = 2,
	YCPOS_CrYCbY = 3
};

/**
 * struct prev_size_params - Structure for size parameters.
 * @hstart: Starting pixel.
 * @vstart: Starting line.
 * @hsize: Width of input image.
 * @vsize: Height of input image.
 * @pixsize: Pixel size of the image in terms of bits.
 * @in_pitch: Line offset of input image.
 * @out_pitch: Line offset of output image.
 */
struct prev_size_params {
	unsigned int hstart;
	unsigned int vstart;
	unsigned int hsize;
	unsigned int vsize;
	unsigned char pixsize;
	unsigned short in_pitch;
	unsigned short out_pitch;
};

/**
 * struct prev_rgb2ycbcr_coeffs - Structure RGB2YCbCr parameters.
 * @coeff: Color conversion gains in 3x3 matrix.
 * @offset: Color conversion offsets.
 */
struct prev_rgb2ycbcr_coeffs {
	short coeff[RGB_MAX][RGB_MAX];
	short offset[RGB_MAX];
};

/**
 * struct prev_darkfrm_params - Structure for Dark frame suppression.
 * @addr: Memory start address.
 * @offset: Line offset.
 */
struct prev_darkfrm_params {
	u32 addr;
	u32 offset;
};

/**
 * struct prev_params - Structure for all configuration
 * @features: Set of features enabled.
 * @cfa: CFA coefficients.
 * @csup: Chroma suppression coefficients.
 * @ytable: Pointer to Luma enhancement coefficients.
 * @nf: Noise filter coefficients.
 * @dcor: Noise filter coefficients.
 * @gtable: Gamma coefficients.
 * @wbal: White Balance parameters.
 * @blk_adj: Black adjustment parameters.
 * @rgb2rgb: RGB blending parameters.
 * @rgb2ycbcr: RGB to ycbcr parameters.
 * @hmf_params: Horizontal median filter.
 * @size_params: Size parameters.
 * @drkf_params: Darkframe parameters.
 * @yclimit: YC limits parameters.
 * @lens_shading_shift:
 * @average: Downsampling rate for averager.
 * @contrast: Contrast.
 * @brightness: Brightness.
 */
struct prev_params {
	u32 features;
	enum preview_ycpos_mode pix_fmt;
	struct ispprev_cfa cfa;
	struct ispprev_csup csup;
	struct ispprev_luma luma;
	struct ispprev_nf nf;
	struct ispprev_dcor dcor;
	struct ispprev_gtables gamma;
	struct ispprev_wbal wbal;
	struct ispprev_blkadj blk_adj;
	struct ispprev_rgbtorgb rgb2rgb;
	struct ispprev_csc rgb2ycbcr;
	struct ispprev_hmed hmed;
	struct prev_size_params size_params;
	struct prev_darkfrm_params drkf_params;
	struct ispprev_yclimit yclimit;
	u32 lens_shading_shift;
	u8 average;
	u8 contrast;
	u8 brightness;
};

/**
 * struct isptables_update - Structure for Table Configuration.
 * @update: Specifies which tables should be updated.
 * @flag: Specifies which tables should be enabled.
 * @prev_nf: Pointer to structure for Noise Filter
 * @lsc: Pointer to LSC gain table. (currently not used)
 * @red_gamma: Pointer to red gamma correction table.
 * @green_gamma: Pointer to green gamma correction table.
 * @blue_gamma: Pointer to blue gamma correction table.
 * @prev_cfa: Pointer to color filter array configuration.
 * @prev_wbal: Pointer to colour and digital gain configuration.
 */
struct isptables_update {
	u32 update;
	u32 flag;
	struct ispprev_nf *nf;
	u32 *lsc;
	struct ispprev_gtables *gamma;
	struct ispprev_cfa *cfa;
	struct ispprev_wbal *wbal;
};

/* Sink and source previewer pads */
#define PREV_PAD_SINK			0
#define PREV_PAD_SOURCE			1
#define PREV_PADS_NUM			2

/*
 * struct isp_prev_device - Structure for storing ISP Preview module information
 * @subdev: V4L2 subdevice
 * @pads: Media entity pads
 * @formats: Probe and active formats at the input pad
 * @input: Module currently connected to the input pad
 * @output: Bitmask of the active output
 * @params: Module configuration data
 * @shadow_update: If set, update the hardware configured in the next interrupt
 * @enabled: Whether the preview engine is enabled
 * @lock: Shadow update lock
 * @update: Bitmask of the parameters to be updated
 * @error: A hardware error occured during capture
 *
 * This structure is used to store the OMAP ISP Preview module Information.
 */
struct isp_prev_device {
	struct v4l2_subdev subdev;
	struct media_entity_pad pads[PREV_PADS_NUM];
	struct v4l2_mbus_framefmt formats[PREV_PADS_NUM][2];

	enum preview_input_entity input;
	unsigned int output;
	struct isp_video video_in;
	struct isp_video video_out;
	unsigned int error;

	struct prev_params params;
	unsigned int shadow_update:1,
		     underrun:1;
	enum isp_pipeline_stream_state state;
	spinlock_t lock;
	u32 update;
};

struct isp_device;

int isp_preview_init(struct isp_device *isp);
void isp_preview_cleanup(struct isp_device *isp);

int isp_preview_register_entities(struct isp_prev_device *prv,
	struct v4l2_device *vdev);
void isp_preview_unregister_entities(struct isp_prev_device *prv);

void isppreview_isr_frame_sync(struct isp_prev_device *prev);
void isppreview_isr(struct isp_prev_device *prev);

void isppreview_config_whitebalance(struct isp_prev_device *isp_prev,
				    const void *prev_wbal);

int isppreview_busy(struct isp_prev_device *isp_prev);

void isppreview_print_status(struct isp_prev_device *isp_prev);

void isppreview_save_context(struct isp_device *isp);

void isppreview_restore_context(struct isp_device *isp);

#endif/* OMAP_ISP_PREVIEW_H */
