/*
 *  Media device support header.
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

#ifndef _MEDIA_DEVICE_H
#define _MEDIA_DEVICE_H

#include <linux/device.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>

#include <media/media-devnode.h>
#include <media/media-entity.h>

/* Each instance of a media device should create the media_device struct,
   either stand-alone or embedded in a larger struct.

   It allows easy access to sub-devices (see v4l2-subdev.h) and provides
   basic media device-level support.
 */

#define MEDIA_DEVICE_NAME_SIZE (20 + 16)

struct media_device {
	/* dev->driver_data points to this struct.
	   Note: dev might be NULL if there is no parent device
	   as is the case with e.g. ISA devices. */
	struct device *dev;
	struct media_devnode devnode;

	u32 entity_id;
	struct list_head entities;

	/* Spinlock is for the entities list. */
	spinlock_t lock;
	/* Mutex for graph changes. */
	struct mutex graph_mutex;

	/* unique device name, by default the driver name + bus ID */
	char name[MEDIA_DEVICE_NAME_SIZE];
};

/* media_devnode to media_device */
#define to_media_device(node) container_of(node, struct media_device, devnode)

int __must_check media_device_register(struct media_device *mdev);
void media_device_unregister(struct media_device *mdev);

int __must_check media_device_register_entity(struct media_device *mdev,
					      struct media_entity *entity);
void media_device_unregister_entity(struct media_entity *entity);

/* Iterate over all entities. */
#define media_device_for_each_entity(entity, mdev)			\
	list_for_each_entry(entity, &(mdev)->entities, list)

#endif
