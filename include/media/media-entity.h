#ifndef _MEDIA_ENTITY_H
#define _MEDIA_ENTITY_H

#include <linux/list.h>
#include <linux/media.h>

struct media_entity_link {
	struct media_entity_pad *source;/* Source pad */
	struct media_entity_pad *sink;	/* Sink pad  */
	struct media_entity_link *other;/* Link in the reverse direction */
	u32 flags;			/* Link flags (MEDIA_LINK_FLAG_*) */
};

struct media_entity_pad {
	struct media_entity *entity;	/* Entity this pad belongs to */
	u32 type;			/* Pad type (MEDIA_PAD_TYPE_*) */
	u32 index;			/* Pad index in the entity pads array */
};

struct media_entity_operations {
	int (*link_setup)(struct media_entity *entity,
			  const struct media_entity_pad *local,
			  const struct media_entity_pad *remote, u32 flags);
	int (*set_power)(struct media_entity *entity, int power);
};

struct media_entity {
	struct list_head list;
	struct media_device *parent;	/* Media device this entity belongs to*/
	u32 id;				/* Entity ID, unique in the parent media
					 * device context */
	const char *name;		/* Entity name */
	u32 type;			/* Entity type (MEDIA_ENTITY_TYPE_*) */
	u32 subtype;			/* Entity subtype (type-specific) */

	u8 num_pads;			/* Number of input and output pads */
	u8 num_links;			/* Number of existing links, both active
					 * and inactive */
	u8 num_backlinks;		/* Number of backlinks */
	u8 max_links;			/* Maximum number of links */

	struct media_entity_pad *pads;	/* Array of pads (num_pads elements) */
	struct media_entity_link *links;/* Array of links (max_links elements)*/

	const struct media_entity_operations *ops;	/* Entity operations */

	int use_count;			/* Use count for the entity. */

	union {
		/* Node specifications */
		struct {
			u32 major;
			u32 minor;
		} v4l;
		struct {
			u32 major;
			u32 minor;
		} fb;
		int alsa;
		int dvb;

		/* Sub-device specifications */
		/* Nothing needed yet */
	};
};

#define MEDIA_ENTITY_ENUM_MAX_DEPTH	16

struct media_entity_graph {
	struct {
		struct media_entity *entity;
		int link;
	} stack[MEDIA_ENTITY_ENUM_MAX_DEPTH];
	int top;
};

extern int media_entity_init(struct media_entity *entity, u8 num_pads,
		struct media_entity_pad *pads, u8 extra_links);
extern void media_entity_cleanup(struct media_entity *entity);
extern int media_entity_create_link(struct media_entity *source, u8 source_pad,
		struct media_entity *sink, u8 sink_pad, u32 flags);
extern int __media_entity_setup_link(struct media_entity_link *link, u32 flags);
extern int media_entity_setup_link(struct media_entity_link *link, u32 flags);
extern struct media_entity_link *media_entity_find_link(
		struct media_entity_pad *source, struct media_entity_pad *sink);
extern struct media_entity_pad *media_entity_remote_pad(
		struct media_entity_pad *pad);

struct media_entity *media_entity_get(struct media_entity *entity);
void media_entity_put(struct media_entity *entity);

void media_entity_graph_walk_start(struct media_entity_graph *graph,
				   struct media_entity *entity);
struct media_entity *
media_entity_graph_walk_next(struct media_entity_graph *graph);

#define media_entity_call(entity, operation, args...)			\
	(((entity)->ops && (entity)->ops->operation) ?			\
	 (entity)->ops->operation((entity) , ##args) : -ENOIOCTLCMD)

#endif

