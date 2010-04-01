/*
 * Stand-Alone Resizer Mem-To-Mem driver.
 *
 * Copyright (C) 2010 Texas Instruments Inc
 * Author: Vaibhav Hiremath <hvaibhav@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/clk.h>
//#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/omap_resizer.h>

#include <plat/iommu.h>
#include <plat/iovmm.h>

#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf-dma-sg.h>

#include "ispreg.h"

#define RESZ_MODULE_NAME "Resz-dev"

MODULE_DESCRIPTION("Stand Alone Resizer (Mem-To-Mem) driver)");
MODULE_AUTHOR("Vaibhav Hiremath <hvaibhav@ti.com>");
MODULE_LICENSE("GPL");

#define BITSET(variable, bit)   ((variable) | (1 << bit))
#define BITRESET(variable, bit) ((variable) & ~(0x00000001 << (bit)))
#define SET_BIT_INPUTRAM        28
#define SET_BIT_CBLIN           29
#define SET_BIT_INPTYP          27
#define SET_BIT_YCPOS           26
#define INPUT_RAM               1
#define UP_RSZ_RATIO            64
#define DOWN_RSZ_RATIO          512
#define UP_RSZ_RATIO1           513
#define DOWN_RSZ_RATIO1         1024
#define RSZ_IN_SIZE_VERT_SHIFT  16
#define MAX_HORZ_PIXEL_8BIT     31
#define MAX_HORZ_PIXEL_16BIT    15

/* Flags that indicate a format can be used for capture/output */
#define MEM2MEM_CAPTURE	(1 << 0)
#define MEM2MEM_OUTPUT	(1 << 1)

#define RESZ_DRV_NAME		"resz-dev"

/* In bytes, per queue */
#define MEM2MEM_VID_MEM_LIMIT	(16 * 1024 * 1024)

/* Default transaction time in msec */
#define MEM2MEM_DEF_TRANSTIME	1000
/* Default number of buffers per transaction */
#define MEM2MEM_DEF_TRANSLEN	1
#define MEM2MEM_COLOR_STEP	(0xff >> 4)
#define MEM2MEM_NUM_TILES	10

#define dprintk(dev, fmt, arg...) \
	v4l2_dbg(0, 1, &dev->v4l2_dev, "%s: " fmt, __func__, ## arg)

/* Resizer register overlay structure */
struct resz_reg {
	u32 rsz_pid;		/* PERIPHERAL ID REGISTER
				 */
	u32 rsz_pcr;		/* PERIPHERAL CONTROL REGISTER
				 */
	u32 rsz_cnt;		/* RESIZER CONTROL REGISTER
				 */
	u32 rsz_out_size;	/* OUTPUT SIZE REGISTER
				 */
	u32 rsz_in_start;	/* INPUT CONFIGURATION REGISTER
				 */
	u32 rsz_in_size;	/* INPUT SIZE REGISTER
				 */
	u32 rsz_sdr_inadd;	/* INPUT ADDRESS REGISTER
				 */
	u32 rsz_sdr_inoff;	/* INPUT OFFSET REGISTER
				 */
	u32 rsz_sdr_outadd;	/* OUTPUT ADDRESS REGISTER
				 */
	u32 rsz_sdr_outoff;	/* OUTPUT OFFSET REGISTER
				 */
	u32 rsz_coeff_horz[16];	/* HORIZONTAL FILTER COEFFICIENTS
				 */
	u32 rsz_coeff_vert[16];	/* VERTICAL FILTER COEFFICIENTS
				 */
	u32 rsz_yehn;		/* LUMINANCE ENHANCER REGISTER
				 */
	u32 sdr_req_exp;	/* SBL READ REQUEST EXPAND
				 */
};

struct resz_config {
	/* Input (Source) frame configuration */
	struct v4l2_pix_format	in_pixfmt;
	/* Output (Destination) frame configuration */
	struct v4l2_pix_format	out_pixfmt;

	/* Common params */
	u32 sizeimage;
	int pixelformat;

	/* Horz/Vert scaling ratio */
	int horz_ratio;
	int vert_ratio;

	/* Horz scaling with interleaved croma enable */
	int cbilin;

	/* Horz/Vert Filter Coeff. */
	struct rsz_coeff coeff;
	int num_htap;		/* 0 = 7tap; 1 = 4tap */
	int num_vtap;		/* 0 = 7tap; 1 = 4tap */

	/* Starting Phase */
	struct rsz_strt_phase start_phase;

	/* Luma Enhancement */
	struct rsz_yenh rsz_yenh;
};

struct resz_fmt {
	char	*name;
	u32	fourcc;          /* v4l2 format id */
	int	depth;
	/* Types the format can be used for */
	u32	types;
};

static struct resz_fmt formats[] = {
	{
		.name	= "4:2:2, packed, UYVY",
		.fourcc	= V4L2_PIX_FMT_UYVY,
		.depth	= 16,
		/* Output-only format */
		.types	= MEM2MEM_OUTPUT | MEM2MEM_CAPTURE,
	},
	{
		.name	= "4:2:2, packed, YUYV",
		.fourcc	= V4L2_PIX_FMT_YUYV,
		.depth	= 16,
		/* Output-only format */
		.types	= MEM2MEM_OUTPUT | MEM2MEM_CAPTURE,
	},
};

/* Per-queue, driver-specific private data */

enum {
	V4L2_M2M_SRC = 0,
	V4L2_M2M_DST = 1,
};

struct resz_dev {
	struct v4l2_device	v4l2_dev;
	struct video_device	*vfd;

	atomic_t		num_inst;
	struct completion	compl_isr;

	spinlock_t		irqlock;
	int			irq;

	struct iommu 		*iommu;
	struct timer_list	timer;

	struct v4l2_m2m_dev	*m2m_dev;
};

struct resz_ctx {
	struct resz_dev	*dev;

	/* Processed buffers in this transaction */
	u8			num_processed;

	/* Transaction length (i.e. how many buffers per transaction) */
	u32			translen;
	/* Transaction time (i.e. simulated processing time) in miliseconds */
	u32			transtime;

	/* Abort requested by m2m */
	int			aborting;

	struct v4l2_m2m_ctx	*m2m_ctx;
	struct resz_config	*resz_config;
	struct resz_reg		register_config;
};

struct resz_buffer {
	/* vb must be first! */
	struct videobuf_buffer	vb;
};

#define V4L2_CID_TRANS_TIME_MSEC	V4L2_CID_PRIVATE_BASE
#define V4L2_CID_TRANS_NUM_BUFS		(V4L2_CID_PRIVATE_BASE + 1)

static struct v4l2_queryctrl resz_ctrls[] = {
	{
		.id		= V4L2_CID_TRANS_TIME_MSEC,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Transaction time (msec)",
		.minimum	= 1,
		.maximum	= 10000,
		.step		= 100,
		.default_value	= 1000,
		.flags		= 0,
	}, {
		.id		= V4L2_CID_TRANS_NUM_BUFS,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Buffers per transaction",
		.minimum	= 1,
		.maximum	= VIDEO_MAX_FRAME,
		.step		= 1,
		.default_value	= 1,
		.flags		= 0,
	},
};

#define NUM_FORMATS ARRAY_SIZE(formats)

static void device_isr(struct resz_dev *dev);

/* Source and destination queue data */

static struct resz_fmt *find_format(struct v4l2_format *f)
{
	struct resz_fmt *fmt;
	u32 k;

	for (k = 0; k < NUM_FORMATS; k++) {
		fmt = &formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat)
			break;
	}

	if (k == NUM_FORMATS)
		return NULL;

	return &formats[k];
}

static struct v4l2_queryctrl *get_ctrl(int id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(resz_ctrls); ++i) {
		if (id == resz_ctrls[i].id) {
			return &resz_ctrls[i];
		}
	}

	return NULL;
}

static inline void rsz_set_exp(unsigned int exp)
{
	omap_writel(((exp & 0x3FF) << 10), OMAP3ISP_SBL_REG(0xF8));
}

static void rsz_hardware_setup(struct resz_reg *register_config)
{
	int cnt;
	int coeffoffset = 0;

	omap_writel(register_config->rsz_cnt,
			OMAP3ISP_RESZ_REG(ISPRSZ_CNT));

	omap_writel(register_config->rsz_in_start,
			OMAP3ISP_RESZ_REG(ISPRSZ_IN_START));
	omap_writel(register_config->rsz_in_size,
			OMAP3ISP_RESZ_REG(ISPRSZ_IN_SIZE));

	omap_writel(register_config->rsz_out_size,
			OMAP3ISP_RESZ_REG(ISPRSZ_OUT_SIZE));
	omap_writel(register_config->rsz_sdr_inadd,
			OMAP3ISP_RESZ_REG(ISPRSZ_SDR_INADD));
	omap_writel(register_config->rsz_sdr_inoff,
			OMAP3ISP_RESZ_REG(ISPRSZ_SDR_INOFF));
	omap_writel(register_config->rsz_sdr_outadd,
			OMAP3ISP_RESZ_REG(ISPRSZ_SDR_OUTADD));
	omap_writel(register_config->rsz_sdr_outoff,
			OMAP3ISP_RESZ_REG(ISPRSZ_SDR_OUTOFF));
	omap_writel(register_config->rsz_yehn,
			OMAP3ISP_RESZ_REG(ISPRSZ_YENH));
	for (cnt = 0; cnt < 16; cnt++)
	{
		omap_writel(register_config->rsz_coeff_horz[cnt],
				OMAP3ISP_RESZ_REG(ISPRSZ_HFILT10 + coeffoffset));

		omap_writel(register_config->rsz_coeff_vert[cnt],
				OMAP3ISP_RESZ_REG(ISPRSZ_VFILT10 + coeffoffset));

		coeffoffset = coeffoffset + 0x04;
	}
	/* Configure the read expand register */
	rsz_set_exp(register_config->sdr_req_exp);

	omap_writel(0x0, OMAP3ISP_REG(ISP_SYSCONFIG));

}
static void resz_enable(struct resz_reg *register_config)
{
	/*Enable Resize operation*/
	register_config->rsz_pcr = 0x1;
	omap_writel(register_config->rsz_pcr,
			OMAP3ISP_RESZ_REG(ISPRSZ_PCR));
}
static void resz_intr_enable(struct resz_reg *register_config, int enable)
{
	u32 intr = 0;

	intr = omap_readl(OMAP3ISP_REG(ISP_IRQ0ENABLE));

	if (enable)
		intr |= IRQ0ENABLE_RSZ_DONE_IRQ;
	else
		intr &= ~IRQ0ENABLE_RSZ_DONE_IRQ;

	omap_writel(-1, OMAP3ISP_REG(ISP_IRQ0STATUS));
	omap_writel(intr, OMAP3ISP_REG(ISP_IRQ0ENABLE));
}
static int device_process(struct resz_ctx *ctx,
			  dma_addr_t in_buf,
			  dma_addr_t out_buf)
{
	struct resz_reg   *register_config = &ctx->register_config;
	int ret = 0;

	register_config->rsz_sdr_inadd = in_buf;
	register_config->rsz_sdr_outadd = out_buf;
	rsz_hardware_setup(register_config);

	resz_intr_enable(register_config, 1);
	resz_enable(register_config);

	ctx->dev->compl_isr.done = 0;
	ret = wait_for_completion_interruptible(&ctx->dev->compl_isr);
	if (ret != 0) {
		dprintk(ctx->dev, "Uexpected exit!!! ret - %d\n", ret);
	}

	resz_intr_enable(register_config, 0);
	device_isr(ctx->dev);

	return 0;
}

static irqreturn_t resz_irq(int irq, void *pdev)
{
	struct resz_dev *resz_dev = pdev;
	struct resz_ctx *curr_ctx;

	curr_ctx = v4l2_m2m_get_curr_priv(resz_dev->m2m_dev);
	if (NULL == curr_ctx) {
		printk("Instance released before end of transaction\n");
		return IRQ_HANDLED;
	}

	/* Ack the Interrupt and disable it */
	resz_intr_enable(&curr_ctx->register_config, 0);

	complete(&(resz_dev->compl_isr));
	return IRQ_HANDLED;
}

/*
 * mem2mem callbacks
 */

/**
 * resz_job_ready - check whether an instance is ready to be scheduled to run
 */
static int resz_job_ready(void *priv)
{
	struct resz_ctx *ctx = priv;

	if (v4l2_m2m_num_src_bufs_ready(ctx->m2m_ctx) < ctx->translen
	    || v4l2_m2m_num_dst_bufs_ready(ctx->m2m_ctx) < ctx->translen) {
		dprintk(ctx->dev, "Not enough buffers available\n");
		return 0;
	}

	return 1;
}

static void resz_job_abort(void *priv)
{
	struct resz_ctx *ctx = priv;

	/* Will cancel the transaction in the next interrupt handler */
	ctx->aborting = 1;
}
static dma_addr_t resz_vmap(struct resz_dev *dev,
		const struct scatterlist *sglist, int sglen)
{
	struct sg_table *sgt;
	struct scatterlist *sg, *src = (struct scatterlist *)sglist;
	unsigned int i, da;
	int err;

	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return -ENOMEM;

	err = sg_alloc_table(sgt, sglen, GFP_KERNEL);
	if (err)
		goto err_sg_alloc;

	for_each_sg(sgt->sgl, sg, sgt->nents, i)
		sg_set_buf(sg, phys_to_virt(sg_dma_address(src + i)),
				sg_dma_len(src + i));

	da = iommu_vmap(dev->iommu, 0, sgt, IOVMF_ENDIAN_LITTLE | IOVMF_ELSZ_8);
	if (IS_ERR_VALUE(da))
		goto err_vmap;
	return (dma_addr_t)da;
err_vmap:
	sg_free_table(sgt);
err_sg_alloc:
	kfree(sgt);
	return -ENOMEM;
}
/* resz_device_run() - prepares and starts the device
 *
 * This simulates all the immediate preparations required
 * before starting a device.
 * This should be called by the framework when it devides to
 * schedule a particular instance.
 */
static void resz_device_run(void *priv)
{
	struct resz_ctx *ctx = priv;
	struct resz_dev *dev = ctx->dev;
	struct resz_buffer *src_buf, *dst_buf;
	struct videobuf_dmabuf *src_dma, *dst_dma;
	struct videobuf_queue *src_vq, *dst_vq;
	unsigned int src_addr, dst_addr;
	int err;


	src_buf = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->m2m_ctx);
	if (!src_buf || !dst_buf)
		printk("Error: unable to get src/dst buffer\n");

	src_dma = videobuf_to_dma(&src_buf->vb);
	src_vq = v4l2_m2m_get_src_vq(ctx->m2m_ctx);
	err = videobuf_dma_map(src_vq, src_dma);
	if (err)
		dprintk(ctx->dev, "Error: unable to map SRC dma\n");

	src_addr = resz_vmap(dev, src_dma->sglist, src_dma->sglen);

	dst_dma = videobuf_to_dma(&dst_buf->vb);
	dst_vq = v4l2_m2m_get_src_vq(ctx->m2m_ctx);
	err = videobuf_dma_map(dst_vq, dst_dma);
	if (err)
		dprintk(ctx->dev, "Error: unable to map DST dma\n");

	dst_addr = resz_vmap(dev, dst_dma->sglist, dst_dma->sglen);

	device_process(ctx, src_addr, dst_addr);
}


static void device_isr(struct resz_dev *resz_dev)
{
	struct resz_ctx *curr_ctx;
	struct resz_buffer *src_buf, *dst_buf;

	curr_ctx = v4l2_m2m_get_curr_priv(resz_dev->m2m_dev);
	if (NULL == curr_ctx) {
		printk("Instance released before end of transaction\n");
		return;
	}

	src_buf = v4l2_m2m_src_buf_remove(curr_ctx->m2m_ctx);
	dst_buf = v4l2_m2m_dst_buf_remove(curr_ctx->m2m_ctx);
	curr_ctx->num_processed++;

	curr_ctx->num_processed = 0;
	v4l2_m2m_job_finish(resz_dev->m2m_dev, curr_ctx->m2m_ctx);
	src_buf->vb.state = dst_buf->vb.state = VIDEOBUF_DONE;
	wake_up(&src_buf->vb.done);
	wake_up(&dst_buf->vb.done);

	return;
}


/*
 * video ioctls
 */

static int vidioc_querycap(struct file *file, void *priv,
			   struct v4l2_capability *cap)
{
	strncpy(cap->driver, RESZ_DRV_NAME, sizeof(cap->driver) - 1);
	strncpy(cap->card, RESZ_DRV_NAME, sizeof(cap->card) - 1);
	cap->bus_info[0] = 0;
	cap->version = KERNEL_VERSION(0, 1, 0);
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_OUTPUT
			  | V4L2_CAP_STREAMING;

	return 0;
}

static int enum_fmt(struct resz_ctx *ctx, struct v4l2_fmtdesc *f, u32 type)
{
	int i, num = 0;

	for (i = 0; i < NUM_FORMATS; ++i) {
		if (formats[i].types & type) {
			/* index-th format of type type found ? */
			if (num == f->index)
				break;
			/* Correct type but haven't reached our index yet,
			 * just increment per-type index */
			++num;
		}
	}

	if (i < NUM_FORMATS) {
		/* Format found */
		strncpy(f->description, formats[i].name, sizeof(f->description) - 1);
		f->pixelformat = formats[i].fourcc;
		return 0;
	}

	/* Format not found */
	return -EINVAL;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(priv, f, MEM2MEM_CAPTURE);
}

static int vidioc_enum_fmt_vid_out(struct file *file, void *priv,
				   struct v4l2_fmtdesc *f)
{
	return enum_fmt(priv, f, MEM2MEM_OUTPUT);
}

static int vidioc_g_fmt(struct resz_ctx *ctx, struct v4l2_format *f)
{
	struct videobuf_queue *vq;
	struct resz_config *resz_config = ctx->resz_config;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		f->fmt.pix.width	= resz_config->out_pixfmt.width;
		f->fmt.pix.height	= resz_config->out_pixfmt.height;
		f->fmt.pix.field	= vq->field;
		f->fmt.pix.pixelformat	= resz_config->out_pixfmt.pixelformat;
		f->fmt.pix.bytesperline	= resz_config->out_pixfmt.bytesperline;
		f->fmt.pix.sizeimage	= resz_config->out_pixfmt.sizeimage;
	} else if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		f->fmt.pix.width	= resz_config->in_pixfmt.width;
		f->fmt.pix.height	= resz_config->in_pixfmt.height;
		f->fmt.pix.field	= vq->field;
		f->fmt.pix.pixelformat	= resz_config->in_pixfmt.pixelformat;
		f->fmt.pix.bytesperline	= resz_config->in_pixfmt.bytesperline;
		f->fmt.pix.sizeimage	= resz_config->in_pixfmt.sizeimage;
	}

	return 0;
}

static int vidioc_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return vidioc_g_fmt(priv, f);
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return vidioc_g_fmt(priv, f);
}

static int vidioc_try_fmt(struct v4l2_format *f, struct resz_fmt *fmt)
{
	enum v4l2_field field;

	field = f->fmt.pix.field;
	if (field == V4L2_FIELD_ANY)
		field = V4L2_FIELD_NONE;
	else if (V4L2_FIELD_NONE != field)
		return -EINVAL;

	/* V4L2 specification suggests the driver corrects the format struct
	 * if any of the dimensions is unsupported */

	if (f->fmt.pix.bytesperline < ((f->fmt.pix.width * fmt->depth) >> 3))
		f->fmt.pix.bytesperline = (f->fmt.pix.width * fmt->depth) >> 3;

	if (f->fmt.pix.sizeimage < (f->fmt.pix.bytesperline * f->fmt.pix.height))
		f->fmt.pix.sizeimage = f->fmt.pix.bytesperline *
			f->fmt.pix.height;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		if (f->fmt.pix.width < 16)
			f->fmt.pix.width = 16;
		if (f->fmt.pix.width > 2047)
			f->fmt.pix.width = 2047;

		if (f->fmt.pix.width % 2) {
			printk("Output H size should be even\n");
			return -EINVAL;
		}

		if (f->fmt.pix.height > 2047)
			f->fmt.pix.height = 2047;
	}

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		if (f->fmt.pix.width < 32)
			f->fmt.pix.width = 32;
	}
	f->fmt.pix.field = field;

	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct resz_fmt *fmt;
	struct resz_ctx *ctx = priv;

	fmt = find_format(f);
	if (!fmt || !(fmt->types & MEM2MEM_CAPTURE)) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	return vidioc_try_fmt(f, fmt);
}

static int vidioc_try_fmt_vid_out(struct file *file, void *priv,
				  struct v4l2_format *f)
{
	struct resz_fmt *fmt;
	struct resz_ctx *ctx = priv;

	fmt = find_format(f);
	if (!fmt || !(fmt->types & MEM2MEM_OUTPUT)) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Fourcc format (0x%08x) invalid.\n",
			 f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	return vidioc_try_fmt(f, fmt);
}


static void rsz_config_ratio(struct resz_config *resz_config,
				struct resz_reg *register_config)
{
	int hsize, vsize, cnt;

	if (resz_config->horz_ratio <= 512)
		hsize = ((32 * resz_config->start_phase.horz_strt_phase +
					(resz_config->out_pixfmt.width - 1) *
					resz_config->horz_ratio + 16) >> 8) + 7;
	else
		hsize = ((64 * resz_config->start_phase.horz_strt_phase +
					(resz_config->out_pixfmt.width - 1) *
					resz_config->horz_ratio + 32) >> 8) + 7;

	if (resz_config->vert_ratio <= 512)
		vsize = ((32 * resz_config->start_phase.vert_strt_phase +
					(resz_config->out_pixfmt.height - 1) *
					resz_config->vert_ratio + 16) >> 8) + 4;
	else
		vsize = ((64 * resz_config->start_phase.vert_strt_phase +
					(resz_config->out_pixfmt.height - 1) *
					resz_config->vert_ratio + 32) >> 8) + 7;

	register_config->rsz_in_size = hsize;

	register_config->rsz_in_size |= ((vsize << ISPRSZ_IN_SIZE_VERT_SHIFT)
			& ISPRSZ_IN_SIZE_VERT_MASK);

	/* This is another workaround for the ISP-MMU translation fault.
	   For the parameters whose image size comes exactly to PAGE_SIZE
	   generates ISP-MMU translation fault. The root-cause is the equation
	   input width = (32*sph + (ow - 1)*hrsz + 16) >> 8 + 7
	   = (64*sph + (ow - 1)*hrsz + 32) >> 8 + 7
	   input height = (32*spv + (oh - 1)*vrsz + 16) >> 8 + 4
	   = (64*spv + (oh - 1)*vrsz + 32) >> 8 + 7

	   we are adjusting the input width to suit for Resizer module,
	   application should use this configuration henceforth.
	 */
	resz_config->in_pixfmt.width = hsize;
	resz_config->in_pixfmt.height = vsize;

	for (cnt = 0; cnt < 16; cnt++) {
		if (resz_config->num_htap) {
			register_config->rsz_coeff_horz[cnt] =
				(resz_config->coeff.tap4filt_coeffs[2 * cnt]
				 & ISPRSZ_HFILT10_COEF0_MASK);
			register_config->rsz_coeff_horz[cnt] |=
				((resz_config->coeff.tap4filt_coeffs[2 * cnt + 1]
				  << ISPRSZ_HFILT10_COEF1_SHIFT)
				 & ISPRSZ_HFILT10_COEF1_MASK);
		} else {
			register_config->rsz_coeff_horz[cnt] =
				(resz_config->coeff.tap7filt_coeffs[2 * cnt]
				 & ISPRSZ_HFILT10_COEF0_MASK);

			register_config->rsz_coeff_horz[cnt] |=
				((resz_config->coeff.tap7filt_coeffs[2 * cnt + 1]
				  << ISPRSZ_HFILT10_COEF1_SHIFT)
				 & ISPRSZ_HFILT10_COEF1_MASK);
		}

		if (resz_config->num_vtap) {
			register_config->rsz_coeff_vert[cnt] =
				(resz_config->coeff.tap4filt_coeffs[2 * cnt]
				 & ISPRSZ_VFILT10_COEF0_MASK);

			register_config->rsz_coeff_vert[cnt] |=
				((resz_config->coeff.tap4filt_coeffs[2 * cnt + 1]
				  << ISPRSZ_VFILT10_COEF1_SHIFT) &
				 ISPRSZ_VFILT10_COEF1_MASK);
		} else {
			register_config->rsz_coeff_vert[cnt] =
				(resz_config->coeff.tap7filt_coeffs[2 * cnt]
				 & ISPRSZ_VFILT10_COEF0_MASK);
			register_config->rsz_coeff_vert[cnt] |=
				((resz_config->coeff.tap7filt_coeffs[2 * cnt + 1]
				  << ISPRSZ_VFILT10_COEF1_SHIFT)
				 & ISPRSZ_VFILT10_COEF1_MASK);
		}
	}
}


static int rsz_set_ratio(struct resz_config *resz_config,
				struct resz_reg *register_config)
{
	int alignment = 0;

	register_config->rsz_cnt = 0;

	if ((resz_config->out_pixfmt.width > 2047) ||
			(resz_config->out_pixfmt.height > 2047)) {
		printk("Invalid output size! - %d", \
					resz_config->out_pixfmt.width);
		goto err_einval;
	}
	if (resz_config->start_phase.horz_strt_phase > 15) {
		printk("Invalid horz starting phase\n");
		goto err_einval;
	}
	if (resz_config->cbilin) {
		register_config->rsz_cnt =
				BITSET(register_config->rsz_cnt, SET_BIT_CBLIN);
	}
	register_config->rsz_cnt = BITSET(register_config->rsz_cnt,
						SET_BIT_INPUTRAM);

	register_config->rsz_cnt = BITRESET(register_config->rsz_cnt,
				SET_BIT_INPTYP);

	if (resz_config->pixelformat == V4L2_PIX_FMT_UYVY)
		register_config->rsz_cnt = BITRESET(register_config->rsz_cnt,
				SET_BIT_YCPOS);
	else if (resz_config->pixelformat == V4L2_PIX_FMT_YUYV)
		register_config->rsz_cnt = BITSET(register_config->rsz_cnt,
					SET_BIT_YCPOS);

	resz_config->vert_ratio = (resz_config->in_pixfmt.height * 256) / resz_config->out_pixfmt.height;
	resz_config->horz_ratio = (resz_config->in_pixfmt.width * 256) / resz_config->out_pixfmt.width;
	if (64 > resz_config->vert_ratio || 64 > resz_config->horz_ratio) {
		printk("Upscaling ratio not supported!");
		goto err_einval;
	}
	resz_config->vert_ratio = (resz_config->in_pixfmt.height - 7) * 256 /
						(resz_config->out_pixfmt.height - 1);
	resz_config->horz_ratio = ((resz_config->in_pixfmt.width - 7) * 256) /
						(resz_config->out_pixfmt.width - 1);

	if (resz_config->horz_ratio <= 512) {
		resz_config->horz_ratio = (resz_config->in_pixfmt.width - 4) * 256 /
						(resz_config->out_pixfmt.width - 1);
		if (resz_config->horz_ratio < 64)
			resz_config->horz_ratio = 64;
		if (resz_config->horz_ratio > 512)
			resz_config->horz_ratio = 512;
		if (resz_config->start_phase.horz_strt_phase > 8)
			goto err_einval;
		resz_config->num_htap = 1;
	} else if (resz_config->horz_ratio >= 513 && resz_config->horz_ratio <= 1024) {
		if (resz_config->start_phase.horz_strt_phase > 4)
			goto err_einval;
		resz_config->num_htap = 0;
	}

	if (resz_config->vert_ratio <= 512) {
		resz_config->vert_ratio = (resz_config->in_pixfmt.height - 4) * 256 /
						(resz_config->out_pixfmt.height - 1);
		if (resz_config->vert_ratio < 64)
			resz_config->vert_ratio = 64;
		if (resz_config->vert_ratio > 512)
			resz_config->vert_ratio = 512;
		if (resz_config->start_phase.vert_strt_phase > 8)
			goto err_einval;
		resz_config->num_vtap = 1;
	} else if (resz_config->vert_ratio >= 513 && resz_config->vert_ratio <= 1024) {
		if (resz_config->start_phase.vert_strt_phase > 4)
			goto err_einval;
		resz_config->num_vtap = 0;
	}

	if ((resz_config->in_pixfmt.bytesperline) % 32) {
		printk( "Invalid input pitch: %d \n",
							resz_config->in_pixfmt.bytesperline);
		goto err_einval;
	}
	if ((resz_config->out_pixfmt.bytesperline) % 32) {
		printk( "Invalid output pitch %d \n",
							resz_config->out_pixfmt.bytesperline);
		goto err_einval;
	}

	if (resz_config->vert_ratio < 256 &&
			(resz_config->in_pixfmt.height < resz_config->out_pixfmt.height)) {
			alignment = (16 / 2);

		if (!(((resz_config->out_pixfmt.width % 2) == 0)
				&& (resz_config->out_pixfmt.width % alignment) == 0)) {
			printk( "wrong hsize\n");
			goto err_einval;
		}
	}
	if (resz_config->horz_ratio >= 64 && resz_config->horz_ratio <= 1024) {
		if (resz_config->out_pixfmt.width > 2047) {
			printk( "wrong width\n");
			goto err_einval;
		}

	} else if (resz_config->horz_ratio > 1024) {
		if (resz_config->out_pixfmt.width > 2047) {
			printk( "wrong width\n");
			goto err_einval;
		}
		if (resz_config->start_phase.horz_strt_phase > 4)
			goto err_einval;
		resz_config->num_htap = 0;
		resz_config->out_pixfmt.width = resz_config->in_pixfmt.width * 256 / 1024;
		if (resz_config->out_pixfmt.width % 32) {
			resz_config->out_pixfmt.width +=
				abs((resz_config->out_pixfmt.width % 32) - 32);
		}
		resz_config->out_pixfmt.bytesperline = resz_config->out_pixfmt.width * 2;
		resz_config->horz_ratio = ((resz_config->in_pixfmt.width - 7) * 256) /
						(resz_config->out_pixfmt.width - 1);

	}

	if (resz_config->vert_ratio > 1024) {
		if (resz_config->out_pixfmt.height > 2047) {
			printk( "wrong width\n");
			goto err_einval;
		}

		resz_config->out_pixfmt.height = resz_config->in_pixfmt.height * 256 / 1024;
		resz_config->vert_ratio = ((resz_config->in_pixfmt.height - 7) * 256) /
						(resz_config->out_pixfmt.height - 1);
		resz_config->num_vtap = 0;

	}
	register_config->rsz_out_size = resz_config->out_pixfmt.width
						& ISPRSZ_OUT_SIZE_HORZ_MASK;

	register_config->rsz_out_size |= (resz_config->out_pixfmt.height
					<< ISPRSZ_OUT_SIZE_VERT_SHIFT)
					& ISPRSZ_OUT_SIZE_VERT_MASK;

	register_config->rsz_sdr_inoff = resz_config->in_pixfmt.bytesperline
					& ISPRSZ_SDR_INOFF_OFFSET_MASK;

	register_config->rsz_sdr_outoff = resz_config->out_pixfmt.bytesperline
					& ISPRSZ_SDR_OUTOFF_OFFSET_MASK;

	if (resz_config->horz_ratio >= 64 && resz_config->horz_ratio <= 512) {
		if (resz_config->start_phase.horz_strt_phase > 8)
			goto err_einval;
	} else if (resz_config->horz_ratio >= 64 && resz_config->horz_ratio <= 512) {
		if (resz_config->start_phase.horz_strt_phase > 4)
			goto err_einval;
	}

	register_config->rsz_cnt |= (resz_config->start_phase.horz_strt_phase
				<< ISPRSZ_CNT_HSTPH_SHIFT)
				& ISPRSZ_CNT_HSTPH_MASK;

	if (resz_config->vert_ratio >= 64 && resz_config->horz_ratio <= 512) {
		if (resz_config->start_phase.vert_strt_phase > 8)
			goto err_einval;
	} else if (resz_config->vert_ratio >= 64 && resz_config->vert_ratio <= 512) {
		if (resz_config->start_phase.vert_strt_phase > 4)
			goto err_einval;
	}

	register_config->rsz_cnt |= (resz_config->start_phase.vert_strt_phase
			<< ISPRSZ_CNT_VSTPH_SHIFT) & ISPRSZ_CNT_VSTPH_MASK;

	register_config->rsz_cnt |= (resz_config->horz_ratio - 1)
				& ISPRSZ_CNT_HRSZ_MASK;

	register_config->rsz_cnt |= ((resz_config->vert_ratio - 1)
			<< ISPRSZ_CNT_VRSZ_SHIFT) & ISPRSZ_CNT_VRSZ_MASK;

	register_config->rsz_in_start |= resz_config->start_phase.horz_strt_phase
				& ISPRSZ_IN_START_HORZ_ST_MASK;

        register_config->rsz_yehn = (resz_config->rsz_yenh.type
					<< ISPRSZ_YENH_ALGO_SHIFT)
					& ISPRSZ_YENH_ALGO_MASK;

	if (resz_config->rsz_yenh.type) {
		register_config->rsz_yehn |= resz_config->rsz_yenh.core
						& ISPRSZ_YENH_CORE_MASK;

		register_config->rsz_yehn |= (resz_config->rsz_yenh.gain
						<< ISPRSZ_YENH_GAIN_SHIFT)
						& ISPRSZ_YENH_GAIN_MASK;

		register_config->rsz_yehn |= (resz_config->rsz_yenh.slop
						<< ISPRSZ_YENH_SLOP_SHIFT);

	}
	/* Default value for read expand:Taken from Davinci */
	register_config->sdr_req_exp = 0;

	rsz_config_ratio(resz_config, register_config);

	return 0;
err_einval:
	return -EINVAL;
}




static int vidioc_s_fmt(struct resz_ctx *ctx, struct v4l2_format *f)
{
	struct videobuf_queue *vq;
	struct resz_fmt *fmt;
	struct resz_config *resz_config = ctx->resz_config;
	struct resz_reg   *register_config = &ctx->register_config;
	int ret = 0;

	vq = v4l2_m2m_get_vq(ctx->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	fmt = find_format(f);
	if (!fmt || !(fmt->types & MEM2MEM_OUTPUT)) {
		v4l2_err(&ctx->dev->v4l2_dev,
				"Fourcc format (0x%08x) invalid.\n",
				f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	if (f->fmt.pix.bytesperline < ((f->fmt.pix.width * fmt->depth) >> 3))
		return -EINVAL;

	if (f->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		if ((f->fmt.pix.width < 16) || (f->fmt.pix.width > 2047) ||
				(f->fmt.pix.height > 2047)) {
			printk("invalid Destination params\n");
			return -EINVAL;
		}

		if (f->fmt.pix.width % 2) {
			printk("Output H size should be even\n");
			return -EINVAL;
		}

		fmt = find_format(f);
		if (!fmt || !(fmt->types & MEM2MEM_CAPTURE)) {
			v4l2_err(&ctx->dev->v4l2_dev,
					"Fourcc format (0x%08x) invalid.\n",
					f->fmt.pix.pixelformat);
			return -EINVAL;
		}
		vidioc_try_fmt(f, fmt);

		vq->field = f->fmt.pix.field;

		/*Copy Configuration data locally*/
		resz_config->out_pixfmt = f->fmt.pix;

		dprintk(ctx->dev,
				"Setting format for type %d, wxh: %dx%d, fmt: %d\n",
				f->type, resz_config->in_pixfmt.width,
				resz_config->in_pixfmt.height, resz_config->pixelformat);

		if (0 != rsz_set_ratio(resz_config, register_config))
			return -EINVAL;
	}

	if (f->type == V4L2_BUF_TYPE_VIDEO_OUTPUT) {
		if (f->fmt.pix.width < 32) {
			printk("invalid Source params\n");
			return -EINVAL;
		}

		vq->field = f->fmt.pix.field;

		fmt = find_format(f);
		if (!fmt || !(fmt->types & MEM2MEM_OUTPUT)) {
			v4l2_err(&ctx->dev->v4l2_dev,
					"Fourcc format (0x%08x) invalid.\n",
					f->fmt.pix.pixelformat);
			return -EINVAL;
		}
		vidioc_try_fmt(f, fmt);

		/*Copy Configuration data locally*/
		resz_config->in_pixfmt = f->fmt.pix;

		dprintk(ctx->dev,
				"Setting format for type %d, wxh: %dx%d, fmt: %d\n",
				f->type, resz_config->in_pixfmt.width,
				resz_config->in_pixfmt.height, resz_config->pixelformat);
	}

	return ret;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;

	ret = vidioc_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	return vidioc_s_fmt(priv, f);
}

static int vidioc_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;

	ret = vidioc_try_fmt_vid_out(file, priv, f);
	if (ret)
		return ret;

	return vidioc_s_fmt(priv, f);
}

static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *reqbufs)
{
	struct resz_ctx *ctx = priv;

	return v4l2_m2m_reqbufs(file, ctx->m2m_ctx, reqbufs);
}

static int vidioc_querybuf(struct file *file, void *priv,
			   struct v4l2_buffer *buf)
{
	struct resz_ctx *ctx = priv;

	return v4l2_m2m_querybuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct resz_ctx *ctx = priv;

	return v4l2_m2m_qbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct resz_ctx *ctx = priv;

	return v4l2_m2m_dqbuf(file, ctx->m2m_ctx, buf);
}

static int vidioc_streamon(struct file *file, void *priv,
			   enum v4l2_buf_type type)
{
	struct resz_ctx *ctx = priv;

	return v4l2_m2m_streamon(file, ctx->m2m_ctx, type);
}

static int vidioc_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct resz_ctx *ctx = priv;

	return v4l2_m2m_streamoff(file, ctx->m2m_ctx, type);
}

static int vidioc_queryctrl(struct file *file, void *priv,
			    struct v4l2_queryctrl *qc)
{
	struct v4l2_queryctrl *c;

	c = get_ctrl(qc->id);
	if (!c)
		return -EINVAL;

	*qc = *c;
	return 0;
}

static int vidioc_g_ctrl(struct file *file, void *priv,
			 struct v4l2_control *ctrl)
{
	struct resz_ctx *ctx = priv;

	switch (ctrl->id) {
	case V4L2_CID_TRANS_TIME_MSEC:
		ctrl->value = ctx->transtime;
		break;

	case V4L2_CID_TRANS_NUM_BUFS:
		ctrl->value = ctx->translen;
		break;

	default:
		v4l2_err(&ctx->dev->v4l2_dev, "Invalid control\n");
		return -EINVAL;
	}

	return 0;
}

static int check_ctrl_val(struct resz_ctx *ctx, struct v4l2_control *ctrl)
{
	struct v4l2_queryctrl *c;

	c = get_ctrl(ctrl->id);
	if (!c)
		return -EINVAL;

	if (ctrl->value < c->minimum
	    || ctrl->value > c->maximum) {
		v4l2_err(&ctx->dev->v4l2_dev, "Value out of range\n");
		return -ERANGE;
	}

	return 0;
}

static int vidioc_s_ctrl(struct file *file, void *priv,
			 struct v4l2_control *ctrl)
{
	struct resz_ctx *ctx = priv;
	int ret = 0;

	ret = check_ctrl_val(ctx, ctrl);
	if (ret != 0)
		return ret;

	switch (ctrl->id) {
	case V4L2_CID_TRANS_TIME_MSEC:
		ctx->transtime = ctrl->value;
		break;

	case V4L2_CID_TRANS_NUM_BUFS:
		ctx->translen = ctrl->value;
		break;

	default:
		v4l2_err(&ctx->dev->v4l2_dev, "Invalid control\n");
		return -EINVAL;
	}

	return 0;
}

static long resz_ioctl(struct file *file, void *priv, int cmd, void *arg)
{
	struct resz_ctx *ctx = priv;
	struct resz_config *resz_config = ctx->resz_config;

	if ((_IOC_TYPE(cmd) != RSZ_IOC_BASE)|| (_IOC_NR(cmd) > RSZ_IOC_MAXNR)) {
		printk("Bad command value \n");
		return -1;
	}

	switch (cmd) {
	case RSZ_S_COEFF:
		{
			struct rsz_coeff *coeff = &resz_config->coeff;

			memcpy(coeff, arg, sizeof(struct rsz_coeff));

			break;
		}
	case RSZ_S_STRT_PHASE:
		{
			struct rsz_strt_phase *phase = &resz_config->start_phase;
			memcpy(phase, arg, sizeof(struct rsz_strt_phase));
			break;
		}
	case RSZ_S_LUMA_ENHANCEMENT:
		{
			struct rsz_yenh *rsz_yenh = &resz_config->rsz_yenh;
			memcpy(rsz_yenh, arg, sizeof(struct rsz_yenh));
			break;
		}
	case RSZ_S_EXP:
		{
			break;
		}
	case RSZ_G_STATUS:
		{
			break;
		}
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ioctl_ops resz_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,

	.vidioc_enum_fmt_vid_cap = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap	= vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap	= vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap	= vidioc_s_fmt_vid_cap,

	.vidioc_enum_fmt_vid_out = vidioc_enum_fmt_vid_out,
	.vidioc_g_fmt_vid_out	= vidioc_g_fmt_vid_out,
	.vidioc_try_fmt_vid_out	= vidioc_try_fmt_vid_out,
	.vidioc_s_fmt_vid_out	= vidioc_s_fmt_vid_out,

	.vidioc_reqbufs		= vidioc_reqbufs,
	.vidioc_querybuf	= vidioc_querybuf,

	.vidioc_qbuf		= vidioc_qbuf,
	.vidioc_dqbuf		= vidioc_dqbuf,

	.vidioc_streamon	= vidioc_streamon,
	.vidioc_streamoff	= vidioc_streamoff,

	.vidioc_queryctrl	= vidioc_queryctrl,
	.vidioc_g_ctrl		= vidioc_g_ctrl,
	.vidioc_s_ctrl		= vidioc_s_ctrl,

	.vidioc_default		= resz_ioctl,
};


/*
 * Queue operations
 */

static void resz_buf_release(struct videobuf_queue *vq,
				struct videobuf_buffer *vb)
{
	struct resz_ctx *ctx = vq->priv_data;

	dprintk(ctx->dev, "type: %d, index: %d, state: %d\n",
		vq->type, vb->i, vb->state);

//	videobuf_vmalloc_free(vb);
	vb->state = VIDEOBUF_NEEDS_INIT;
}

static int resz_buf_setup(struct videobuf_queue *vq, unsigned int *count,
			  unsigned int *size)
{
	struct resz_ctx *ctx = vq->priv_data;
	struct resz_config *resz_config = ctx->resz_config;

	if (resz_config->in_pixfmt.sizeimage > resz_config->out_pixfmt.sizeimage)
		*size = resz_config->in_pixfmt.sizeimage;
	else
		*size = resz_config->out_pixfmt.sizeimage;

	resz_config->sizeimage = *size;
	dprintk(ctx->dev, "size:%d, Src WxH: %dx%d, Dst WxH: %dx%d\n",
		*size, resz_config->in_pixfmt.width,
		resz_config->in_pixfmt.height, resz_config->out_pixfmt.width,
		resz_config->out_pixfmt.height);

	if (0 == *count)
		*count = VIDEO_MAX_FRAME;

	while (*size * *count > MEM2MEM_VID_MEM_LIMIT)
		(*count)--;

	v4l2_info(&ctx->dev->v4l2_dev,
		  "%d buffers of size %d set up.\n", *count, *size);

	return 0;
}

static int resz_buf_prepare(struct videobuf_queue *vq,
			       struct videobuf_buffer *vb,
			       enum v4l2_field field)
{
	struct resz_ctx *ctx = vq->priv_data;
	struct resz_config *resz_config = ctx->resz_config;
	int ret;

	dprintk(ctx->dev, "type: %d, index: %d, state: %d\n",
		vq->type, vb->i, vb->state);

	if (vb->baddr) {
		/* User-provided buffer */
		if (vb->bsize < resz_config->sizeimage) {
			/* Buffer too small to fit a frame */
			v4l2_err(&ctx->dev->v4l2_dev,
				 "User-provided buffer too small (%d < %d)\n",
				 resz_config->sizeimage, vb->bsize);
			return -EINVAL;
		}
	} else if (vb->state != VIDEOBUF_NEEDS_INIT
			&& vb->bsize < resz_config->sizeimage) {
		/* We provide the buffer, but it's already been inited
		 * and is too small */
		return -EINVAL;
	}

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		vb->width	= resz_config->out_pixfmt.width;
		vb->height	= resz_config->out_pixfmt.height;
		vb->bytesperline = resz_config->out_pixfmt.bytesperline;
	} else {
		vb->width	= resz_config->in_pixfmt.width;
		vb->height	= resz_config->in_pixfmt.height;
		vb->bytesperline = resz_config->in_pixfmt.bytesperline;
	}

	vb->size	= resz_config->sizeimage;
	vb->field	= field;

	if (VIDEOBUF_NEEDS_INIT == vb->state) {
		ret = videobuf_iolock(vq, vb, NULL);
		if (ret) {
			v4l2_err(&ctx->dev->v4l2_dev,
				 "Iolock failed\n");
			goto fail;
		}

	}

	vb->state = VIDEOBUF_PREPARED;

	return 0;
fail:
	resz_buf_release(vq, vb);
	return ret;
}

static void resz_buf_queue(struct videobuf_queue *vq,
			   struct videobuf_buffer *vb)
{
	struct resz_ctx *ctx = vq->priv_data;

	v4l2_m2m_buf_queue(ctx->m2m_ctx, vq, vb);
}

static struct videobuf_queue_ops resz_qops = {
	.buf_setup	= resz_buf_setup,
	.buf_prepare	= resz_buf_prepare,
	.buf_queue	= resz_buf_queue,
	.buf_release	= resz_buf_release,
};

static void queue_init(void *priv, struct videobuf_queue *vq,
		       enum v4l2_buf_type type)
{
	struct resz_ctx *ctx = priv;

	videobuf_queue_sg_init(vq, &resz_qops, ctx->dev->v4l2_dev.dev,
				    &ctx->dev->irqlock, type, V4L2_FIELD_NONE,
				    sizeof(struct resz_buffer), priv);
}

/*
 * File operations
 */
static int resz_open(struct file *file)
{
	struct resz_dev *dev = video_drvdata(file);
	struct resz_ctx *ctx = NULL;
	struct resz_config *resz_config;

	ctx = kzalloc(sizeof *ctx, GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	resz_config = kzalloc(sizeof(struct resz_config), GFP_KERNEL);
	if (!resz_config) {
		kfree(ctx);
		return -ENOMEM;
	}
	ctx->resz_config = resz_config;

	file->private_data = ctx;
	ctx->dev = dev;
	ctx->translen = MEM2MEM_DEF_TRANSLEN;
	ctx->transtime = MEM2MEM_DEF_TRANSTIME;
	ctx->num_processed = 0;

	ctx->m2m_ctx = v4l2_m2m_ctx_init(ctx, dev->m2m_dev, queue_init);
	if (IS_ERR(ctx->m2m_ctx)) {
		kfree(resz_config);
		kfree(ctx);
		return PTR_ERR(ctx->m2m_ctx);
	}

	atomic_inc(&dev->num_inst);

	dprintk(dev, "Created instance %p, m2m_ctx: %p\n", ctx, ctx->m2m_ctx);

	return 0;
}

static int resz_release(struct file *file)
{
	struct resz_dev *dev = video_drvdata(file);
	struct resz_ctx *ctx = file->private_data;

	dprintk(dev, "Releasing instance %p\n", ctx);

	v4l2_m2m_ctx_release(ctx->m2m_ctx);
	kfree(ctx->resz_config);
	kfree(ctx);

	atomic_dec(&dev->num_inst);

	return 0;
}

static unsigned int resz_poll(struct file *file,
				 struct poll_table_struct *wait)
{
	struct resz_ctx *ctx = (struct resz_ctx *)file->private_data;

	return v4l2_m2m_poll(file, ctx->m2m_ctx, wait);
}

static int resz_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct resz_ctx *ctx = (struct resz_ctx *)file->private_data;

	return v4l2_m2m_mmap(file, ctx->m2m_ctx, vma);
}

static const struct v4l2_file_operations resz_fops = {
	.owner		= THIS_MODULE,
	.open		= resz_open,
	.release	= resz_release,
	.poll		= resz_poll,
	.ioctl		= video_ioctl2,
	.mmap		= resz_mmap,
};

static struct video_device resz_videodev = {
	.name		= RESZ_DRV_NAME,
	.fops		= &resz_fops,
	.ioctl_ops	= &resz_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,
};

static struct v4l2_m2m_ops resz_m2m_ops = {
	.device_run	= resz_device_run,
	.job_ready	= resz_job_ready,
	.job_abort	= resz_job_abort,
};

struct clk * cam_ick, *cam_mclk, *l3_ick;
static void isp_enable_all_clocks(void)
{
	cam_ick = clk_get(NULL, "cam_ick");
	if (IS_ERR(cam_ick)) {
		printk("Failed to get cam_ick\n");
	}
	clk_enable(cam_ick);
	cam_mclk = clk_get(NULL, "cam_mclk");
	if (IS_ERR(cam_mclk)) {
		printk("Failed to get cam_ick\n");
	}
	clk_enable(cam_mclk);
	l3_ick = clk_get(NULL, "l3_ick");
	if (IS_ERR(l3_ick)) {
		printk("Failed to get cam_ick\n");
	}
	clk_enable(l3_ick);
}

static void isp_disable_all_clocks(void)
{
	clk_disable(l3_ick);
	clk_disable(cam_mclk);
	clk_disable(cam_ick);
	clk_put(l3_ick);
	clk_put(cam_mclk);
	clk_put(cam_ick);
}

static int resz_probe(struct platform_device *pdev)
{
	struct resz_dev *rsz_dev;
	struct video_device *vfd;
	int ret;

	rsz_dev = kzalloc(sizeof *rsz_dev, GFP_KERNEL);
	if (!rsz_dev)
		return -ENOMEM;

	spin_lock_init(&rsz_dev->irqlock);

	ret = v4l2_device_register(&pdev->dev, &rsz_dev->v4l2_dev);
	if (ret)
		goto free_dev;

	atomic_set(&rsz_dev->num_inst, 0);
	init_completion(&rsz_dev->compl_isr);
	vfd = video_device_alloc();
	if (!vfd) {
		v4l2_err(&rsz_dev->v4l2_dev, "Failed to allocate video device\n");
		goto unreg_dev;
	}

	*vfd = resz_videodev;

	ret = video_register_device(vfd, VFL_TYPE_GRABBER, 0);
	if (ret) {
		v4l2_err(&rsz_dev->v4l2_dev, "Failed to register video device\n");
		goto rel_vdev;
	}

	video_set_drvdata(vfd, rsz_dev);
	snprintf(vfd->name, sizeof(vfd->name), "%s", resz_videodev.name);
	rsz_dev->vfd = vfd;
	v4l2_info(&rsz_dev->v4l2_dev, RESZ_MODULE_NAME
			"Device registered as /dev/video%d\n", vfd->num);

	platform_set_drvdata(pdev, rsz_dev);

	rsz_dev->m2m_dev = v4l2_m2m_init(&resz_m2m_ops);
	if (IS_ERR(rsz_dev->m2m_dev)) {
		v4l2_err(&rsz_dev->v4l2_dev, "Failed to init mem2mem device\n");
		ret = PTR_ERR(rsz_dev->m2m_dev);
		goto err_m2m;
	}

	/* Request IRQ */
	rsz_dev->irq = 24;
	ret = request_irq(rsz_dev->irq, resz_irq, IRQF_SHARED,
			"omap3isp", rsz_dev);

	if (ret)
		printk("Failed to get intr: ret - %d\n", ret);
	/* Enable all clocks */
	isp_enable_all_clocks();

	rsz_dev->iommu = iommu_get("isp");
	if (IS_ERR(rsz_dev->iommu)) {
		printk("Failed to get IOMMU\n");
		rsz_dev->iommu = NULL;
	}

	return 0;

err_m2m:
	video_unregister_device(rsz_dev->vfd);
rel_vdev:
	video_device_release(vfd);
unreg_dev:
	v4l2_device_unregister(&rsz_dev->v4l2_dev);
free_dev:
	kfree(rsz_dev);

	return ret;
}

static int resz_remove(struct platform_device *pdev)
{
	struct resz_dev *dev =
		(struct resz_dev *)platform_get_drvdata(pdev);

	v4l2_info(&dev->v4l2_dev, "Removing " RESZ_MODULE_NAME);
	v4l2_m2m_release(dev->m2m_dev);
	del_timer_sync(&dev->timer);
	video_unregister_device(dev->vfd);
	v4l2_device_unregister(&dev->v4l2_dev);
	kfree(dev);

	isp_disable_all_clocks();
	return 0;
}

void resz_dev_release(struct device *dev)
{
}

static struct platform_device resz_pdev = {
	.name		= RESZ_DRV_NAME,
	.dev.release	= resz_dev_release,
};

static struct platform_driver resz_pdrv = {
	.probe		= resz_probe,
	.remove		= resz_remove,
	.driver		= {
		.name	= RESZ_DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static void __exit resz_exit(void)
{
	platform_driver_unregister(&resz_pdrv);
	platform_device_unregister(&resz_pdev);
}

static int __init resz_init(void)
{
	int ret;

	ret = platform_device_register(&resz_pdev);
	if (ret)
		return ret;

	ret = platform_driver_register(&resz_pdrv);
	if (ret)
		platform_device_unregister(&resz_pdev);

	return ret;
}

module_init(resz_init);
module_exit(resz_exit);

