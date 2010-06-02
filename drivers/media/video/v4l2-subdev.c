/*
 *  V4L2 subdevice support.
 *
 *  Copyright (C) 2009  Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/videodev2.h>
#include <linux/slab.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-event.h>

static int subdev_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct media_entity *entity;
	struct v4l2_fh *vfh;
	int ret;

	if (!sd->initialized)
		return -EAGAIN;

	vfh = kzalloc(sizeof(*vfh), GFP_KERNEL);
	if (vfh == NULL)
		return -ENOMEM;

	ret = v4l2_fh_init(vfh, vdev);
	if (ret)
		goto err;

	if (sd->flags & V4L2_SUBDEV_USES_EVENTS) {
		ret = v4l2_event_init(vfh);
		if (ret)
			goto err;

		ret = v4l2_event_alloc(vfh, sd->nevents);
		if (ret)
			goto err;
	}

	v4l2_fh_add(vfh);
	file->private_data = vfh;

	entity = media_entity_get(&sd->entity);
	if (!entity) {
		ret = -EBUSY;
		goto err;
	}

	return 0;

err:
	v4l2_fh_del(vfh);
	v4l2_fh_exit(vfh);
	kfree(vfh);

	return ret;
}

static int subdev_close(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct v4l2_fh *vfh = file->private_data;

	media_entity_put(&sd->entity);

	v4l2_fh_del(vfh);
	v4l2_fh_exit(vfh);
	kfree(vfh);

	return 0;
}

static long subdev_do_ioctl(struct file *file, unsigned int cmd, void *arg)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct v4l2_fh *fh = file->private_data;

	switch (cmd) {
	case VIDIOC_QUERYCTRL:
		return v4l2_subdev_call(sd, core, queryctrl, arg);

	case VIDIOC_QUERYMENU:
		return v4l2_subdev_call(sd, core, querymenu, arg);

	case VIDIOC_G_CTRL:
		return v4l2_subdev_call(sd, core, g_ctrl, arg);

	case VIDIOC_S_CTRL:
		return v4l2_subdev_call(sd, core, s_ctrl, arg);

	case VIDIOC_G_EXT_CTRLS:
		return v4l2_subdev_call(sd, core, g_ext_ctrls, arg);

	case VIDIOC_S_EXT_CTRLS:
		return v4l2_subdev_call(sd, core, s_ext_ctrls, arg);

	case VIDIOC_TRY_EXT_CTRLS:
		return v4l2_subdev_call(sd, core, try_ext_ctrls, arg);

	case VIDIOC_DQEVENT:
		if (!(sd->flags & V4L2_SUBDEV_USES_EVENTS))
			return -ENOIOCTLCMD;

		return v4l2_event_dequeue(fh, arg, file->f_flags & O_NONBLOCK);

	case VIDIOC_SUBSCRIBE_EVENT:
		return v4l2_subdev_call(sd, core, subscribe_event, fh, arg);

	case VIDIOC_UNSUBSCRIBE_EVENT:
		return v4l2_subdev_call(sd, core, unsubscribe_event, fh, arg);

	case VIDIOC_SUBDEV_G_FMT: {
		struct v4l2_subdev_pad_format *format = arg;
		return v4l2_subdev_call(sd, pad, get_fmt, format->pad,
					&format->format, format->which);
	}

	case VIDIOC_SUBDEV_S_FMT: {
		struct v4l2_subdev_pad_format *format = arg;
		return v4l2_subdev_call(sd, pad, set_fmt, format->pad,
					&format->format, format->which);
	}

	case VIDIOC_SUBDEV_G_FRAME_INTERVAL:
		return v4l2_subdev_call(sd, video, g_frame_interval, arg);

	case VIDIOC_SUBDEV_S_FRAME_INTERVAL:
		return v4l2_subdev_call(sd, video, s_frame_interval, arg);

	case VIDIOC_SUBDEV_ENUM_MBUS_CODE:
		return v4l2_subdev_call(sd, pad, enum_mbus_code, arg);

	default:
		return v4l2_subdev_call(sd, core, ioctl, cmd, arg);
	}

	return 0;
}

static long subdev_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	return video_usercopy(file, cmd, arg, subdev_do_ioctl);
}

static unsigned int subdev_poll(struct file *file, poll_table *wait)
{
	struct video_device *vdev = video_devdata(file);
	struct v4l2_subdev *sd = vdev_to_v4l2_subdev(vdev);
	struct v4l2_fh *fh = file->private_data;

	if (!(sd->flags & V4L2_SUBDEV_USES_EVENTS))
		return POLLERR;

	poll_wait(file, &fh->events->wait, wait);

	if (v4l2_event_pending(fh))
		return POLLPRI;

	return 0;
}

const struct v4l2_file_operations v4l2_subdev_fops = {
	.owner = THIS_MODULE,
	.open = subdev_open,
	.unlocked_ioctl = subdev_ioctl,
	.release = subdev_close,
	.poll = subdev_poll,
};

void v4l2_subdev_init(struct v4l2_subdev *sd, const struct v4l2_subdev_ops *ops)
{
	INIT_LIST_HEAD(&sd->list);
	/* ops->core MUST be set */
	BUG_ON(!ops || !ops->core);
	sd->ops = ops;
	sd->flags = 0;
	sd->name[0] = '\0';
	sd->grp_id = 0;
	sd->priv = NULL;
	sd->entity.name = sd->name;
	sd->entity.type = MEDIA_ENTITY_TYPE_SUBDEV;
	sd->initialized = 1;
}
EXPORT_SYMBOL(v4l2_subdev_init);

int v4l2_subdev_set_power(struct media_entity *entity, int power)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);

	dev_dbg(entity->parent->dev,
		"%s power%s\n", entity->name, power ? "on" : "off");

	return v4l2_subdev_call(sd, core, s_power, power);
}
EXPORT_SYMBOL_GPL(v4l2_subdev_set_power);
