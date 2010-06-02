#ifndef __ISP_VIDEO_H
#define __ISP_VIDEO_H

#include <linux/version.h>
#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-fh.h>

#include "ispqueue.h"

#define ISP_VIDEO_DRIVER_NAME		"ispvideo"
#define ISP_VIDEO_DRIVER_VERSION	KERNEL_VERSION(0, 0, 1)

struct isp_device;
struct isp_video;
struct v4l2_mbus_framefmt;
struct v4l2_pix_format;

enum isp_pipeline_stream_state {
	ISP_PIPELINE_STREAM_STOPPED = 0,
	ISP_PIPELINE_STREAM_CONTINUOUS = 1,
	ISP_PIPELINE_STREAM_SINGLESHOT = 2,
};

enum isp_pipeline_state {
	/* The stream has been started on the input video node. */
	ISP_PIPELINE_STREAM_INPUT = 1,
	/* The stream has been started on the output video node. */
	ISP_PIPELINE_STREAM_OUTPUT = 2,
	/* At least one buffer is queued on the input video node. */
	ISP_PIPELINE_QUEUE_INPUT = 4,
	/* At least one buffer is queued on the output video node. */
	ISP_PIPELINE_QUEUE_OUTPUT = 8,
	/* The pipeline is currently streaming. */
	ISP_PIPELINE_STREAM = 16,
};

struct isp_pipeline {
	spinlock_t lock;
	unsigned int state;
	struct isp_video *input;
	struct isp_video *output;
};

static inline int isp_pipeline_ready(struct isp_pipeline *pipe)
{
	return pipe->state == (ISP_PIPELINE_STREAM_INPUT |
			       ISP_PIPELINE_STREAM_OUTPUT |
			       ISP_PIPELINE_QUEUE_INPUT |
			       ISP_PIPELINE_QUEUE_OUTPUT);
}

/*
 * struct isp_buffer - ISP buffer
 * @buffer: ISP video buffer
 * @isp_addr: MMU mapped address (a.k.a. device address) of the buffer.
 */
struct isp_buffer {
	struct isp_video_buffer buffer;
	dma_addr_t isp_addr;
};

#define to_isp_buffer(buf)	container_of(buf, struct isp_buffer, buffer)

/*
 * struct isp_video_operations - ISP video operations
 * @init:	Initialise the pipeline. Called when the first user opens the
 * 		device node. For legacy video nodes only.
 * @cleanup:	Clean up the pipeline. Called when the last user closes the
 * 		device node. For legacy video nodes only.
 * @stream_off:	Stop video streaming in the pipeline. Called on
 *		VIDIOC_STREAMOFF. For legacy video nodes only.
 * @queue:	Resume streaming when a buffer is queued. Called on VIDIOC_QBUF
 * 		if there was no buffer previously queued.
 */
struct isp_video_operations {
	int(*init)(struct isp_video *video);
	int(*cleanup)(struct isp_video *video);
	int(*stream_off)(struct isp_video *video);
	int(*queue)(struct isp_video *video, struct isp_buffer *buffer);
};

struct isp_video {
	struct video_device video;
	enum v4l2_buf_type type;
	struct media_entity_pad pad;

	struct mutex mutex;
	atomic_t active;
	atomic_t users;

	struct isp_device *isp;

	unsigned int capture_mem;
	unsigned int alignment;

	/* Pipeline state */
	struct isp_pipeline __pipe;
	struct isp_pipeline *pipe;
	struct mutex stream_lock;

	/* Video buffers queue */
	struct isp_video_queue *queue;
	struct list_head dmaqueue;
	atomic_t sequence;

	const struct isp_video_operations *ops;
	const struct v4l2_ioctl_ops *ioctl_ops;
};

#define to_isp_video(vdev)	container_of(vdev, struct isp_video, video)

struct isp_video_fh {
	struct v4l2_fh vfh;
	struct isp_video *video;
	struct isp_video_queue queue;
	struct v4l2_format format;
};

#define to_isp_video_fh(fh)	container_of(fh, struct isp_video_fh, vfh)
#define isp_video_queue_to_isp_video_fh(q) \
				container_of(q, struct isp_video_fh, queue)

extern int isp_video_init(struct isp_video *video, const char *name);
extern int isp_video_register(struct isp_video *video,
			      struct v4l2_device *vdev);
extern void isp_video_unregister(struct isp_video *video);
extern struct isp_buffer *isp_video_buffer_next(struct isp_video *video,
						unsigned int error);

extern struct media_entity_pad *isp_video_remote_pad(struct isp_video *video);
extern void isp_video_mbus_to_pix(const struct isp_video *video,
				  const struct v4l2_mbus_framefmt *mbus,
				  struct v4l2_pix_format *pix);
extern void isp_video_pix_to_mbus(const struct v4l2_pix_format *pix,
				  struct v4l2_mbus_framefmt *mbus);

#endif /* __ISP_VIDEO_H */

