/*
 * Media device node handling
 *
 * Copyright (C) 2009  Laurent Pinchart <laurent.pinchart@ideasonboard.com>
 *
 * Common functions for media-related drivers to register and unregister media
 * device nodes.
 */
#ifndef _MEDIA_DEVNODE_H
#define _MEDIA_DEVNODE_H

#include <linux/poll.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>

/* Media device node type. */
#define MEDIA_TYPE_DEVICE	0
#define MEDIA_TYPE_MAX		1

/*
 * Flag to mark the media_devnode struct as registered. Drivers must not touch
 * this flag directly, it will be set and cleared by media_devnode_register and
 * media_devnode_unregister.
 */
#define MEDIA_FLAG_REGISTERED	0

struct media_file_operations {
	struct module *owner;
	ssize_t (*read) (struct file *, char __user *, size_t, loff_t *);
	ssize_t (*write) (struct file *, const char __user *, size_t, loff_t *);
	unsigned int (*poll) (struct file *, struct poll_table_struct *);
	long (*ioctl) (struct file *, unsigned int, unsigned long);
	long (*unlocked_ioctl) (struct file *, unsigned int, unsigned long);
	unsigned long (*get_unmapped_area) (struct file *, unsigned long,
				unsigned long, unsigned long, unsigned long);
	int (*mmap) (struct file *, struct vm_area_struct *);
	int (*open) (struct file *);
	int (*release) (struct file *);
};

/**
 * struct media_devnode - Media device node
 * @parent:	parent device
 * @name:	media device node name
 * @type:	node type, one of the MEDIA_TYPE_* constants
 * @minor:	device node minor number
 * @num:	device node number
 * @flags:	flags, combination of the MEDIA_FLAG_* constants
 *
 * This structure represents a media-related device node.
 *
 * The @parent is a physical device. It must be set by core or device drivers
 * before registering the node.
 *
 * @name is a descriptive name exported through sysfs. It doesn't have to be
 * unique.
 *
 * The device node number @num is used to create the kobject name and thus
 * serves as a hint to udev when creating the device node.
 */
struct media_devnode {
	/* device ops */
	const struct media_file_operations *fops;

	/* sysfs */
	struct device dev;		/* v4l device */
	struct cdev *cdev;		/* character device */
	struct device *parent;		/* device parent */

	/* device info */
	char name[32];
	int type;

	int minor;
	u16 num;
	unsigned long flags;		/* Use bitops to access flags */

	/* callbacks */
	void (*release)(struct media_devnode *mdev);
};

/* dev to media_devnode */
#define to_media_devnode(cd) container_of(cd, struct media_devnode, dev)

int __must_check media_devnode_register(struct media_devnode *mdev, int type);
void media_devnode_unregister(struct media_devnode *mdev);

const char *media_devnode_type_name(int type);
struct media_devnode *media_devnode_data(struct file *file);

static inline int media_devnode_is_registered(struct media_devnode *mdev)
{
	return test_bit(MEDIA_FLAG_REGISTERED, &mdev->flags);
}

#endif /* _MEDIA_DEVNODE_H */
