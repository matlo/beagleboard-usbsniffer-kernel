/*
 *  Media device support.
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
#include <linux/media.h>

#include <media/media-device.h>
#include <media/media-devnode.h>
#include <media/media-entity.h>

static int media_device_open(struct file *filp)
{
	return 0;
}

static int media_device_close(struct file *filp)
{
	return 0;
}

static struct media_entity *find_entity(struct media_device *mdev, u32 id)
{
	struct media_entity *entity;

	spin_lock(&mdev->lock);

	media_device_for_each_entity(entity, mdev) {
		if (entity->id == id) {
			spin_unlock(&mdev->lock);
			return entity;
		}
	}

	spin_unlock(&mdev->lock);

	return NULL;
}

static long media_device_enum_entities(struct media_device *mdev,
				       struct media_user_entity __user *uent)
{
	struct media_entity *ent;
	struct media_user_entity u_ent;

	if (copy_from_user(&u_ent.id, &uent->id, sizeof(u_ent.id)))
		return -EFAULT;

	ent = find_entity(mdev, u_ent.id);

	if (ent == NULL)
		return -EINVAL;

	u_ent.name[0] = '\0';
	if (ent->name)
		strlcpy(u_ent.name, ent->name, sizeof(u_ent.name));
	u_ent.type = ent->type;
	u_ent.subtype = ent->subtype;
	u_ent.pads = ent->num_pads;
	u_ent.links = ent->num_links - ent->num_backlinks;
	u_ent.v4l.major = ent->v4l.major;
	u_ent.v4l.minor = ent->v4l.minor;
	if (copy_to_user(uent, &u_ent, sizeof(u_ent)))
		return -EFAULT;
	return 0;
}

static void media_device_kpad_to_upad(const struct media_entity_pad *kpad,
				      struct media_user_pad *upad)
{
	upad->entity = kpad->entity->id;
	upad->index = kpad->index;
	upad->type = kpad->type;
}

static long media_device_enum_links(struct media_device *mdev,
				    struct media_user_links __user *ulinks)
{
	struct media_entity *entity;
	struct media_user_links links;

	if (copy_from_user(&links, ulinks, sizeof(links)))
		return -EFAULT;

	entity = find_entity(mdev, links.entity);
	if (entity == NULL)
		return -EINVAL;

	if (links.pads) {
		unsigned int p;

		for (p = 0; p < entity->num_pads; p++) {
			struct media_user_pad pad;
			media_device_kpad_to_upad(&entity->pads[p], &pad);
			if (copy_to_user(&links.pads[p], &pad, sizeof(pad)))
				return -EFAULT;
		}
	}

	if (links.links) {
		struct media_user_link __user *ulink;
		unsigned int l;

		for (l = 0, ulink = links.links; l < entity->num_links; l++) {
			struct media_user_link link;

			/* Ignore backlinks. */
			if (entity->links[l].source->entity != entity)
				continue;

			media_device_kpad_to_upad(entity->links[l].source,
						  &link.source);
			media_device_kpad_to_upad(entity->links[l].sink,
						  &link.sink);
			link.flags = entity->links[l].flags;
			if (copy_to_user(ulink, &link, sizeof(*ulink)))
				return -EFAULT;
			ulink++;
		}
	}
	if (copy_to_user(ulinks, &links, sizeof(*ulinks)))
		return -EFAULT;
	return 0;
}

static long media_device_setup_link(struct media_device *mdev,
				    struct media_user_link __user *_ulink)
{
	struct media_entity_link *link = NULL;
	struct media_user_link ulink;
	struct media_entity *source;
	struct media_entity *sink;
	int ret;

	if (copy_from_user(&ulink, _ulink, sizeof(ulink)))
		return -EFAULT;

	/* Find the source and sink entities and link.
	 */
	source = find_entity(mdev, ulink.source.entity);
	sink = find_entity(mdev, ulink.sink.entity);

	if (source == NULL || sink == NULL)
		return -EINVAL;

	if (ulink.source.index >= source->num_pads ||
	    ulink.sink.index >= sink->num_pads)
		return -EINVAL;

	link = media_entity_find_link(&source->pads[ulink.source.index],
				      &sink->pads[ulink.sink.index]);
	if (link == NULL)
		return -EINVAL;

	/* Setup the link on both entities. */
	ret = __media_entity_setup_link(link, ulink.flags);

	if (copy_to_user(_ulink, &ulink, sizeof(ulink)))
		return -EFAULT;

	return ret;
}

static long media_device_ioctl(struct file *filp, unsigned int cmd,
			       unsigned long arg)
{
	struct media_devnode *devnode = media_devnode_data(filp);
	struct media_device *dev = to_media_device(devnode);
	long ret;

	switch (cmd) {
	case MEDIA_IOC_ENUM_ENTITIES:
		ret = media_device_enum_entities(dev,
				(struct media_user_entity __user *)arg);
		break;

	case MEDIA_IOC_ENUM_LINKS:
		mutex_lock(&dev->graph_mutex);
		ret = media_device_enum_links(dev,
				(struct media_user_links __user *)arg);
		mutex_unlock(&dev->graph_mutex);
		break;

	case MEDIA_IOC_SETUP_LINK:
		mutex_lock(&dev->graph_mutex);
		ret = media_device_setup_link(dev,
				(struct media_user_link __user *)arg);
		mutex_unlock(&dev->graph_mutex);
		break;

	default:
		ret = -ENOIOCTLCMD;
	}

	return ret;
}

static const struct media_file_operations media_device_fops = {
	.owner = THIS_MODULE,
	.open = media_device_open,
	.unlocked_ioctl = media_device_ioctl,
	.release = media_device_close,
};

static void media_device_release(struct media_devnode *mdev)
{
}

/**
 * media_device_register - register a media device
 * @mdev:	The media device
 *
 * The caller is responsible for initializing the media device before
 * registration. The following fields must be set:
 *
 * - dev should point to the parent device. The field can be NULL when no
 *   parent device is available (for instance with ISA devices).
 * - name should be set to the device name. If the name is empty a parent
 *   device must be set. In that case the name will be set to the parent
 *   device driver name followed by a space and the parent device name.
 */
int __must_check media_device_register(struct media_device *mdev)
{
	mdev->entity_id = 1;
	INIT_LIST_HEAD(&mdev->entities);
	spin_lock_init(&mdev->lock);
	mutex_init(&mdev->graph_mutex);

	/* If dev == NULL, then name must be filled in by the caller */
	if (mdev->dev == NULL && WARN_ON(!mdev->name[0]))
		return 0;

	/* Set name to driver name + device name if it is empty. */
	if (!mdev->name[0])
		snprintf(mdev->name, sizeof(mdev->name), "%s %s",
			mdev->dev->driver->name, dev_name(mdev->dev));

	/* Register the device node. */
	mdev->devnode.fops = &media_device_fops;
	mdev->devnode.parent = mdev->dev;
	strlcpy(mdev->devnode.name, mdev->name, sizeof(mdev->devnode.name));
	mdev->devnode.release = media_device_release;
	return media_devnode_register(&mdev->devnode, MEDIA_TYPE_DEVICE);
}
EXPORT_SYMBOL_GPL(media_device_register);

/**
 * media_device_unregister - unregister a media device
 * @mdev:	The media device
 *
 */
void media_device_unregister(struct media_device *mdev)
{
	struct media_entity *entity;
	struct media_entity *next;

	list_for_each_entry_safe(entity, next, &mdev->entities, list)
		media_device_unregister_entity(entity);

	media_devnode_unregister(&mdev->devnode);
}
EXPORT_SYMBOL_GPL(media_device_unregister);

/**
 * media_device_register_entity - Register an entity with a media device
 * @mdev:	The media device
 * @entity:	The entity
 */
int __must_check media_device_register_entity(struct media_device *mdev,
					      struct media_entity *entity)
{
	/* Warn if we apparently re-register an entity */
	WARN_ON(entity->parent != NULL);
	entity->parent = mdev;

	spin_lock(&mdev->lock);
	entity->id = mdev->entity_id++;
	list_add_tail(&entity->list, &mdev->entities);
	spin_unlock(&mdev->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(media_device_register_entity);

/**
 * media_device_unregister_entity - Unregister an entity
 * @entity:	The entity
 *
 * If the entity has never been registered this function will return
 * immediately.
 */
void media_device_unregister_entity(struct media_entity *entity)
{
	struct media_device *mdev = entity->parent;

	if (mdev == NULL)
		return;

	spin_lock(&mdev->lock);
	list_del(&entity->list);
	spin_unlock(&mdev->lock);
	entity->parent = NULL;
}
EXPORT_SYMBOL_GPL(media_device_unregister_entity);

