#ifndef __LINUX_MEDIA_H
#define __LINUX_MEDIA_H

#define MEDIA_ENTITY_TYPE_NODE		1
#define MEDIA_ENTITY_TYPE_SUBDEV	2

#define MEDIA_NODE_TYPE_V4L		1
#define MEDIA_NODE_TYPE_FB		2
#define MEDIA_NODE_TYPE_ALSA		3
#define MEDIA_NODE_TYPE_DVB		4

#define MEDIA_SUBDEV_TYPE_VID_DECODER	1
#define MEDIA_SUBDEV_TYPE_VID_ENCODER	2
#define MEDIA_SUBDEV_TYPE_MISC		3

#define MEDIA_PAD_TYPE_INPUT		1
#define MEDIA_PAD_TYPE_OUTPUT		2

#define MEDIA_LINK_FLAG_ACTIVE		(1 << 0)
#define MEDIA_LINK_FLAG_IMMUTABLE	(1 << 1)

struct media_user_pad {
	__u32 entity;	/* entity ID */
	__u32 index;	/* pad index */
	__u32 type;	/* pad type */
};

struct media_user_entity {
	__u32 id;
	char name[32];
	__u32 type;
	__u32 subtype;
	__u8 pads;
	__u32 links;

	union {
		/* Node specifications */
		struct {
			__u32 major;
			__u32 minor;
		} v4l;
		struct {
			__u32 major;
			__u32 minor;
		} fb;
		int alsa;
		int dvb;

		/* Sub-device specifications */
		/* Nothing needed yet */
	};
};

struct media_user_link {
	struct media_user_pad source;
	struct media_user_pad sink;
	__u32 flags;
};

struct media_user_links {
	__u32 entity;
	/* Should have enough room for pads elements */
	struct media_user_pad __user *pads;
	/* Should have enough room for links elements */
	struct media_user_link __user *links;
};

#define MEDIA_IOC_ENUM_ENTITIES		_IOWR('M', 1, struct media_user_entity)
#define MEDIA_IOC_ENUM_LINKS		_IOWR('M', 2, struct media_user_links)
#define MEDIA_IOC_SETUP_LINK		_IOWR('M', 3, struct media_user_link)

#endif /* __LINUX_MEDIA_H */

