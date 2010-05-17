/*
 * proxy.c -- Proxy gadget driver
 *
 * Copyright (C) 2010 Nicolas Boichat
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/utsname.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/cdev.h>

#include <asm/byteorder.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <asm/system.h>
#include <linux/uaccess.h>
#include <asm/unaligned.h>

#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include <linux/usb.h>

#include "gadget_chips.h"

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"

/*-------------------------------------------------------------------------*/

#define USB_MAXCONFIG                  8       /* Arbitrary limit */

#define DRIVER_DESC		"Proxy Gadget"
#define DRIVER_VERSION		"2010 ..."

static const char shortname [] = "gadgetproxy";
static const char driver_desc [] = DRIVER_DESC;

/*-------------------------------------------------------------------------*/

struct proxy_endpoint {
	struct usb_endpoint_descriptor* desc;

	/* Gadget side */
	struct usb_ep		*gadget_ep;
	struct usb_request	*req;

	/* Device side */
	struct urb              *urb;

	int flying;
};

struct proxy_dev {
	spinlock_t		lock;		/* lock this structure (FIXME: useless for now) */
	struct usb_gadget	*gadget;

	struct usb_device	*udev;			/* the usb device for this device */

 	/* EP0 stuff */
	struct usb_ctrlrequest  *ctrlreq;
	struct usb_request	*req;

	/* EP1 stuff */
	struct proxy_endpoint   *ep1;

	struct proxy_endpoint   *ep2;
};

static struct proxy_dev usb_proxy_gadget;

static int device_setup_ep(struct proxy_dev *dev, struct proxy_endpoint* ep);
static void device_setup_complete(struct urb *urb);
static void device_ep2_irq(struct urb *urb);

/*-------------------------------------------------------------------------*/

static int debug = 24;

#define xprintk(d, level, fmt, args...) \
	printk(level "%s: " fmt, DRIVER_DESC, ## args)

#define DBG(level, dev, fmt, args...)					\
	{ if (debug >= level) xprintk(dev, KERN_DEBUG, fmt, ## args); }

#define ERROR(dev, fmt, args...) \
	xprintk(dev, KERN_ERR, fmt, ## args)
#define WARNING(dev, fmt, args...) \
	xprintk(dev, KERN_WARNING, fmt, ## args)
#define INFO(dev, fmt, args...) \
	xprintk(dev, KERN_INFO, fmt, ## args)

/*-------------------------------------------------------------------------*/

#define USB_DESC_BUFSIZE    512

/*-------------------------------------------------------------------------*/
/* Gadget side of the driver */

/* Allocate and free usb_request (gadget equivalent of urbs) */
static struct usb_request *
proxy_req_alloc(struct usb_ep *ep, unsigned len, gfp_t gfp_flags)
{
	struct usb_request	*req;

	req = usb_ep_alloc_request(ep, gfp_flags);

	//DBG(usb_proxy_gadget, "proxy_req_alloc(len=%d), got %p\n", len, req);

	if (req != NULL) {
		req->length = len;
		req->buf = kmalloc(len, gfp_flags);
		if (req->buf == NULL) {
			usb_ep_free_request(ep, req);
			return NULL;
		}
	}

	return req;
}

static void
proxy_req_free(struct usb_ep *ep, struct usb_request *req)
{
	if (ep != NULL && req != NULL) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

/* EP operations callbacks */

static void proxy_setup_complete(struct usb_ep *ep, struct usb_request *req)
{
	DBG(20, (struct proxy_dev *) ep->driver_data,
	    "setup complete --> %d, %d/%d\n",
	    req->status, req->actual, req->length);

/*	if (req->status || req->actual != req->length)
		DBG(20, (struct proxy_dev *) ep->driver_data,
				"setup complete --> %d, %d/%d\n",
				req->status, req->actual, req->length);*/
}

static void proxy_ep1_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct proxy_dev * dev = ep->driver_data;
	int status;

	DBG(30, dev,
	    "EP1 complete --> %d, %d/%d (flying:%d)\n",
	    req->status, req->actual, req->length,
		dev->ep1->flying);

	status = usb_submit_urb (dev->ep1->urb, GFP_ATOMIC);
	if (status)
		ERROR(dev, "can't resubmit intr, status %d\n", status);
	dev->ep1->flying -= 1;

	//proxy_req_free(ep, req);
}

static void proxy_ep2_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct proxy_dev	*dev = ep->driver_data;
	int status;

	DBG(25, dev,
	    "EP2 complete --> %d, %d/%d\n",
	    req->status, req->actual, req->length);
	
	switch (req->status) {
	case 0:			/* success */
		break;
	case -ECONNRESET:	/* unlink */
	case -ENOENT:
	case -ESHUTDOWN:
		ERROR(dev, "EP2 error: status %d, not resubmitting\n", req->status);
		return;
	/* -EPIPE:  should clear the halt */
	default:		/* error */
		ERROR(dev, "EP2 error: status %d, resubmitting\n", req->status);
		goto resubmit;
	}

	memcpy(dev->ep2->urb->transfer_buffer, req->buf, req->actual);
	dev->ep2->urb->transfer_buffer_length = req->actual;

	status = usb_submit_urb(dev->ep2->urb, GFP_ATOMIC);
	DBG(25, dev, "EP2 submit urb --> %d\n", status);
	/* TODO: check status */

	return;

resubmit:
	status = usb_ep_queue(dev->ep2->gadget_ep, req, GFP_ATOMIC);
	//DBG(dev, "ep2_queue --> %d\n", status);
	if (status < 0) {
		req->status = 0;
	}
}


/*-------------------------------------------------------------------------*/

/*
 * The setup() callback implements all the ep0 functionality that's not
 * handled lower down.
 */
static int
proxy_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *ctrl)
{
	struct proxy_dev	*dev = get_gadget_data(gadget);
	struct usb_request	*req = dev->req;
	int			value;
	u16			wIndex = le16_to_cpu(ctrl->wIndex);
	u16			wValue = le16_to_cpu(ctrl->wValue);
	u16			wLength = le16_to_cpu(ctrl->wLength);

	struct urb *urb;

	unsigned int pipe;

	DBG(19, dev, "ctrl req%02x(%02x).%02x v%04x i%04x l%d\n",
	    ctrl->bRequestType, ctrl->bRequestType&USB_TYPE_MASK,
	    ctrl->bRequest, wValue, wIndex, wLength);

	req->complete = proxy_setup_complete;

	if (ctrl->bRequestType & USB_DIR_IN) {
		pipe = usb_rcvctrlpipe(dev->udev, 0);
		
		DBG(20, dev, "DIR_IN (%p %d)\n", req->buf, req->length);
	} else {
		pipe = usb_sndctrlpipe(dev->udev, 0);
		
		DBG(20, dev, "DIR_OUT\n");
	}
		
	urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!urb)
		return -ENOMEM;
		
	/* ctrl is allocated on the stack in MUSB,
	   so copy it beforehand. */
	memcpy(dev->ctrlreq, ctrl, sizeof(*dev->ctrlreq));

	/* FIXME: check buf and length (there could be some overruns) */
	/* FIXME: we may need to copy the data, or req->buf may be overridden
	 * if 2 control requests follow one another */
	usb_fill_control_urb(urb, dev->udev, pipe, (void*)dev->ctrlreq,
			     req->buf, wLength,
			     device_setup_complete, gadget);
	/* TODO: Understand how the DMA works... */
	//urb->setup_dma = usbhid->cr_dma;
		
	value = usb_submit_urb(urb, GFP_ATOMIC);
	
	DBG(20, dev, "URB submitted (%d)...\n", value);
	return 0;
}

static void
proxy_disconnect(struct usb_gadget *gadget)
{
	struct proxy_dev	*dev = get_gadget_data(gadget);
	unsigned long		flags;

	DBG(5, dev, "%s\n", __func__);

	spin_lock_irqsave(&dev->lock, flags);

	/* TODO: Do something! */

	spin_unlock_irqrestore(&dev->lock, flags);
}

static void
proxy_unbind(struct usb_gadget *gadget)
{
	struct proxy_dev	*dev = get_gadget_data(gadget);


	DBG(5, dev, "%s\n", __func__);

	if (dev->req) {
		proxy_req_free(gadget->ep0, dev->req);
		dev->req = NULL;
	}

	/* FIXME: Ideally, we should dequeue reqs... */

	if (dev->ep1) {
		usb_ep_fifo_flush(dev->ep1->gadget_ep);
		usb_ep_disable(dev->ep1->gadget_ep);
	}

	if (dev->ep2) {
		usb_ep_fifo_flush(dev->ep2->gadget_ep);
		usb_ep_disable(dev->ep2->gadget_ep);
	}

	set_gadget_data(gadget, NULL);
}

static int __init
proxy_bind(struct usb_gadget *gadget)
{
	struct proxy_dev	*dev;
	int			status = -ENOMEM;

	DBG(5, dev, "%s\n", __func__);

	dev = &usb_proxy_gadget;

	if (gadget_is_sa1100(gadget)) {
		/* hardware can't write zero length packets. */
		ERROR(dev, "SA1100 controller is unsupport by this driver\n");
		goto fail;
	}

	/* all we really need is bulk IN/OUT (FIXME: what does that mean?) */
	usb_ep_autoconfig_reset(gadget);

	spin_lock_init(&dev->lock);

	/* preallocate control message data and buffer */
	dev->req = proxy_req_alloc(gadget->ep0, USB_DESC_BUFSIZE,
			GFP_KERNEL);
	if (!dev->req) {
		status = -ENOMEM;
		goto fail;
	}

	dev->req->complete = proxy_setup_complete;

	/* finish hookup to lower layer ... */
	dev->gadget = gadget;
	set_gadget_data(gadget, dev);
	gadget->ep0->driver_data = dev;

	INFO(dev, "%s, version: " DRIVER_VERSION "\n", driver_desc);
	INFO(dev, "using %s\n", gadget->name);

	return 0;

fail:
	proxy_unbind(gadget);
	return status;
}

/*---*/

static struct usb_gadget_driver proxy_gadget_driver = {
	.speed		= USB_SPEED_FULL, /* FIXME: this should be dynamic */

	.function	= (char *) driver_desc,
	.bind		= proxy_bind,
	.unbind		= proxy_unbind,

	.setup		= proxy_setup,
	.disconnect	= proxy_disconnect,

	.driver		= {
		.name		= (char *) shortname,
		.owner		= THIS_MODULE,
	},
};

/*-------------------------------------------------------------------------*/
/* USB device part */

static void rewrite_config(unsigned char* buffer, int length) {
	struct proxy_dev *dev = &usb_proxy_gadget;
	struct usb_descriptor_header *header;
	struct usb_endpoint_descriptor *d;
	int ninterval;

	DBG(10, dev, "Rewriting configuration...\n");

	while (length > 0) {
		if (length < sizeof(struct usb_descriptor_header)) {
			WARN(dev, "Some excess bytes at the end of config descriptor?!?\n");
			break;
		}

		header = (struct usb_descriptor_header *) buffer;

		length -= header->bLength;

		if (length < 0) {
			WARN(dev, "Incomplete config descriptor?!?\n");
			break;
		}

		DBG(15, dev, "Descriptor type %x\n", header->bDescriptorType);

		if (header->bDescriptorType == USB_DT_ENDPOINT) {
			d = (struct usb_endpoint_descriptor *) buffer;

			/* Fixes bInterval for LS devices (measured in ms).
			 * But HS is measured in microframes (0.125ms),
			 * and polling interval is 2**(bInterval-1) => 2**(7-1)*0.125 ms = 8 ms.
			 * For FS: 2**(4-1)*1 ms = 8 ms
			 * See USB specs p.299 (Table 9-13). */

			if (dev->udev->speed == USB_SPEED_LOW) {
				/* FIXME: FS/HS doesn't seem to change
				 * anything on my PC, and everything is still
				 * defined in microframes (1/8 ms)*/
				ninterval = ilog2(d->bInterval*8)+1;
				DBG(15, dev, "Updated bInterval %d->%d\n", d->bInterval, ninterval);
				d->bInterval = ninterval;
			}
		}

		buffer += header->bLength;
	}
}

static void dump_endpoint(struct usb_endpoint_descriptor* desc) {
	DBG(15, dev, "Endpoint: %02x (%s %s)\n",
	    desc->bEndpointAddress,
	    usb_endpoint_dir_in(desc) ? "IN": "OUT",
	    usb_endpoint_xfer_bulk(desc) ? "bulk" :
	    usb_endpoint_xfer_control(desc) ? "control" :
	    usb_endpoint_xfer_int(desc) ? "int" :
	    usb_endpoint_xfer_isoc(desc) ? "isoc" :
	    "???");
}

static struct usb_ep *find_gadget_endpoint(struct usb_gadget *gadget, 
					   struct usb_endpoint_descriptor* desc) {
	struct proxy_dev *dev = get_gadget_data(gadget);
	struct usb_ep *ep = NULL;

	char myname[16];

	/* FIXME: This code is horrible, but it seems
	 * to be the way the gadget framework works. */
	snprintf(myname, 16, "ep%d%s",
		 desc->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK,
		 ((desc->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) ?
		 "in": "out");

	DBG(15, dev, "autoconf: Looking for endpoint %s\n", myname);

	list_for_each_entry (ep, &gadget->ep_list, ep_list) {
		if (!strcmp(ep->name, myname)) {
			return ep;
		}
	}

	return NULL;
}

static void bridge_endpoints(struct usb_gadget *gadget, int index) {
	struct proxy_dev	*dev = get_gadget_data(gadget);
	int value;
	int i, j, k;
	struct usb_host_config* config;
	struct usb_host_interface *alt = NULL;

	dev->ep1 = kzalloc(sizeof(struct proxy_endpoint), GFP_ATOMIC);
	dev->ep2 = kzalloc(sizeof(struct proxy_endpoint), GFP_ATOMIC);

	DBG(10, dev, "Bridging endpoints.\n");

	for (i = 0; i < dev->udev->descriptor.bNumConfigurations; i++) {
		config = &dev->udev->config[i];
		DBG(15, dev, "Config %d (%d)\n",
		    i, config->desc.bConfigurationValue);
		if (config->desc.bConfigurationValue == index) {
			DBG(15, dev, "Got the config!\n");
			for (j = 0; j < config->desc.bNumInterfaces; j++) {
				/* Set up endpoints for alternate interface setting 0 */
				alt = usb_find_alt_setting(config, i, 0);
				if (!alt)
					/* No alt setting 0? Pick the first setting. */
					alt = &config->intf_cache[i]->altsetting[0];

				DBG(15, dev, "Interface %d: %d endpoints\n", j, alt->desc.bNumEndpoints);

				for (k = 0; k < alt->desc.bNumEndpoints; k++) {
					dump_endpoint(&alt->endpoint[k].desc);
					if (alt->endpoint[k].desc.bEndpointAddress == 0x81) {
						dev->ep1->desc = &alt->endpoint[k].desc;
					}
					if (alt->endpoint[k].desc.bEndpointAddress == 0x02) {
						dev->ep2->desc = &alt->endpoint[k].desc;
					}
					
				}
			}
			break;
		}
	}

	if (!dev->ep1->desc) {
		ERROR(dev, "Cannot find EP1!\n");
	} else {
		u16 wMaxPacketSize = le16_to_cpu(dev->ep1->desc->wMaxPacketSize);

		/* Init EP1 on the gagdet side... */
		/* Note: usb_ep_autoconfig does not look at bInterval,
		 * so it should be ok to use the non-rewritten descriptor. */
		//dev->ep1->gadget_ep = usb_ep_autoconfig(gadget, dev->ep1->desc);
		dev->ep1->gadget_ep = find_gadget_endpoint(gadget, dev->ep1->desc);
		if (!dev->ep1->gadget_ep) {
			dev_err(&gadget->dev, "can't autoconfigure on %s\n",
				gadget->name);
			return;
		}
		dev->ep1->gadget_ep->driver_data = dev;	/* claim */
	
		value = usb_ep_enable(dev->ep1->gadget_ep, dev->ep1->desc);
		if (value)
			ERROR(dev, "can't enable %s, result %d\n", dev->ep1->gadget_ep->name, value);
	
		dev->ep1->req = proxy_req_alloc(dev->ep1->gadget_ep, wMaxPacketSize,
				      GFP_KERNEL);
		if (!dev->ep1->req) {
			ERROR(dev, "Cannot allocate request.\n");
			return;
		}

		dev->ep1->req->complete = proxy_ep1_complete;
		dev->ep1->req->context = dev;

		/* ... and on the device side */
		device_setup_ep(dev, dev->ep1);
	}

	if (!dev->ep2->desc) {
		ERROR(dev, "Cannot find EP2!\n");
	} else {
		/* Autoconfig may change the endpoint address, not what we want! */
		//dev->ep2->gadget_ep = usb_ep_autoconfig(gadget, dev->ep2->desc);
		dev->ep2->gadget_ep = find_gadget_endpoint(gadget, dev->ep2->desc);
		if (!dev->ep2->gadget_ep) {
			dev_err(&gadget->dev, "can't autoconfigure on %s\n",
				gadget->name);
			return;
		}
		dev->ep2->gadget_ep->driver_data = dev;	/* claim */
	
		value = usb_ep_enable(dev->ep2->gadget_ep, dev->ep2->desc);
		if (value)
			ERROR(dev, "can't enable %s, result %d\n", dev->ep2->gadget_ep->name, value);

		/* Setup on the device side */
		device_setup_ep(dev, dev->ep2);

		dev->ep2->req = proxy_req_alloc(dev->ep2->gadget_ep, 64,
				      GFP_KERNEL);
		if (!dev->ep2->req) {
			ERROR(dev, "Cannot allocate request.\n");
		}

		dev->ep2->req->complete = proxy_ep2_complete;
		dev->ep2->req->context = dev;

		dev->ep2->req->length = 64;
		dev->ep2->req->zero = 0;
	
		value = usb_ep_queue(dev->ep2->gadget_ep, dev->ep2->req, GFP_ATOMIC);
		DBG(25, dev, "ep2_queue --> %d\n", value);
		if (value < 0) {
			dev->ep2->req->status = 0;
			proxy_ep2_complete(dev->ep2->gadget_ep, dev->ep2->req);
		}
	}

}

/* Callback, when a ctrl request has come back from the device */
static void device_setup_complete(struct urb *urb) {
	struct usb_gadget    *gadget = urb->context; 
	struct proxy_dev	*dev = get_gadget_data(gadget);
	struct usb_request	*req = dev->req;
	struct usb_ctrlrequest *ctrl = dev->ctrlreq;
	u16			wValue = le16_to_cpu(ctrl->wValue);
	int			value = -EOPNOTSUPP;
	int                     index;

	DBG(20, dev, "Device setup complete (%d; %d)!",
	    urb->status,
	    urb->actual_length);

	DBG(20, dev, "ctrl req%02x(%02x).%02x v%04x\n",
	    ctrl->bRequestType, ctrl->bRequestType&USB_TYPE_MASK,
	    ctrl->bRequest, wValue);

	if ((ctrl->bRequestType == USB_DIR_IN) &&
	    (ctrl->bRequest == USB_REQ_GET_DESCRIPTOR)) {
		DBG(15, dev, "GET_DESCRIPTOR (%04x)\n", wValue);
		switch (wValue >> 8) {
		case USB_DT_DEVICE:
			DBG(15, dev, "DT_DEVICE\n");
			/* TODO: In theory, I should not update that field, but
			 * MUSB forces High-speed, which means 64 bytes.
			 * In Full-speed, we could support 8, 16, 32, 64 bytes
			 * (8 is what low-speed needs).
			 * See USB 2.0 specs, 5.5.3. */

			((struct usb_device_descriptor*)req->buf)->bMaxPacketSize0
				= gadget->ep0->maxpacket;
			break;
		case USB_DT_CONFIG:
			index = wValue & 0xFF;
			DBG(15, dev, "DT_CONFIG (%d)\n", index);
			if (index >= dev->udev->descriptor.bNumConfigurations) {
				ERROR(dev, "Config %d, while only %d were expected!\n",
				      index, dev->udev->descriptor.bNumConfigurations);
				break;
			}

			value = usb_parse_configuration(&dev->udev->dev, index,
				   &dev->udev->config[index], req->buf,
				   urb->actual_length);
			if (value < 0) {
				ERROR(dev, "Parse configuration error\n");
			}
			rewrite_config(req->buf, urb->actual_length);
			break;
		}
	}

	/* TODO: Not sure what to do if urb->status < 0... */
	req->length = urb->actual_length;
	req->zero = 0; //value < wLength;
	value = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
	DBG(20, dev, "ep_queue --> %d\n", urb->status);
	if (value < 0) {
		req->status = 0;
		proxy_setup_complete(gadget->ep0, req);
	}

	/* If SET_CONFIGURATION has been sent to the host,
	 * initialize the other endpoints. */
	/* TODO: Handle multiple configurations */
	if (((ctrl->bRequestType&USB_TYPE_MASK) == USB_TYPE_STANDARD) &&
	    (ctrl->bRequest == USB_REQ_SET_CONFIGURATION) ) {
		DBG(15, dev, "SET_CONFIGURATION\n");

		/* FIXME: Handle reconfiguration */

		//if (dev->udev->state != USB_STATE_CONFIGURED) {

			/* The kernel refuses to send any non-control urbs if
			 * udev->state is not CONFIGURED */
			/* TODO: Maybe we should call the standard usb_set_configuration,
			 * to initialize the dev properly. */
			/* TODO: What about stuff like bandwidth allocation done
			 * in usb_set_configuration?!? */
			//usb_set_configuration(dev->udev, XXX);
		        /* FIXME: usb_hcd_alloc_bandwidth also seems to do quite a bit
			 * of useful stuff, including add/drop_endpoint. */
			dev->udev->state = USB_STATE_CONFIGURED;

			bridge_endpoints(gadget, wValue & 0xFF);
		//}
	}
}

/* EP1IN (interrupt) callback */
static void device_ep1_irq(struct urb *urb)
{
	struct proxy_dev *dev = urb->context;
	//struct usb_device *udev = dev->udev;
	int status;	

	switch (urb->status) {
	case 0:			/* success */
		break;
	case -ECONNRESET:	/* unlink */
	case -ENOENT:
	case -ESHUTDOWN:
		ERROR(dev, "EP1 error: status %d, not resubmitting\n", urb->status);
		return;
	/* -EPIPE:  should clear the halt */
	default:		/* error */
		ERROR(dev, "EP1 error: status %d, resubmitting\n", urb->status);
		goto resubmit;
	}

	DBG(30, dev, "Got an EP1 urb back (%d)!\n", urb->actual_length);

	memcpy(dev->ep1->req->buf, urb->transfer_buffer, urb->actual_length);

	dev->ep1->req->length = urb->actual_length;
	dev->ep1->req->zero = 0;
	
	dev->ep1->flying += 1;
	status = usb_ep_queue(dev->ep1->gadget_ep, dev->ep1->req, GFP_ATOMIC);
	DBG(30, dev, "ep_queue --> %d\n", status);
	if (status < 0) {
		dev->ep1->req->status = 0;
		proxy_ep1_complete(dev->ep1->gadget_ep, dev->ep1->req);
	}

	return;

resubmit:
	; /* Resubmission is done in proxy_ep1_complete normally */
	status = usb_submit_urb (urb, GFP_ATOMIC);
	if (status)
		ERROR(dev, "can't resubmit urb, status %d\n", status);
}

static void device_ep2_irq(struct urb *urb) {
	struct proxy_dev *dev = urb->context;
	int status;

	DBG(25, dev, "EP2 IRQ callback\n");

	status = usb_ep_queue(dev->ep2->gadget_ep, dev->ep2->req, GFP_ATOMIC);
	//DBG(dev, "ep2_queue --> %d\n", status);
	if (status < 0) {
		dev->ep2->req->status = 0;
	}
}

/* Connect EP1 on the device side */
static int device_setup_ep(struct proxy_dev *dev, struct proxy_endpoint* ep) {
	struct usb_device *udev = dev->udev;
	int pipe;
	int status;
	u16 wMaxPacketSize = le16_to_cpu(ep->desc->wMaxPacketSize);
	void* data;

	INFO(dev, "Connecting EP...\n");

	ep->urb = usb_alloc_urb(0, GFP_KERNEL);
	/* TODO: Sanity check */

	data = usb_buffer_alloc(udev, wMaxPacketSize, GFP_ATOMIC, &ep->urb->transfer_dma);
	/* TODO: Sanity check */

	/* FIXME: fix that, and setup IRQ handler properly */
	if (usb_endpoint_xfer_int(ep->desc)) {
		pipe = usb_rcvintpipe(udev, ep->desc->bEndpointAddress);

		/* Setup an urb on the device side for EP1 */
		usb_fill_int_urb(ep->urb, udev, pipe,
				 data, wMaxPacketSize,
				 device_ep1_irq, dev, ep->desc->bInterval);
	} else if (usb_endpoint_xfer_bulk(ep->desc)) {
		if ((ep->desc->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
			pipe = usb_rcvbulkpipe(udev, ep->desc->bEndpointAddress);

			/* Setup an urb on the device side for EP1 */
			usb_fill_bulk_urb(ep->urb, udev, pipe,
					  data, wMaxPacketSize,
					  device_ep1_irq, dev);		
		} else {
			pipe = usb_sndbulkpipe(udev, ep->desc->bEndpointAddress);

			/* Setup an urb on the device side for EP2 */
			usb_fill_bulk_urb(ep->urb, udev, pipe,
					  data, wMaxPacketSize,
					  device_ep2_irq, dev);
		}
	} else {
		ERROR(dev, "Unknown transfer type for EP\n");
		return 1;
	}

	ep->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;


	if ((ep->desc->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
		status = usb_submit_urb(dev->ep1->urb, GFP_ATOMIC);

		if (status) {
			ERROR(dev, "can't submit EP urb, status %d\n", status);
			/* TODO: Do something... */
		}
	}

	return 0;
}

static int device_probe(struct usb_device *udev) {
	int status;
	struct proxy_dev *dev = &usb_proxy_gadget;
	int ncfg = udev->descriptor.bNumConfigurations;

	INFO(dev, "device_probe (%04x:%04x)\n",
	     le16_to_cpu(udev->descriptor.idVendor), le16_to_cpu(udev->descriptor.idProduct));

	if (dev->udev) {
		INFO(dev, "Already attached to another device!\n");
		return -1;
	}

	dev->ctrlreq = kmalloc(sizeof(*dev->ctrlreq), GFP_KERNEL);

	if (ncfg > USB_MAXCONFIG) {
		WARN(dev, "too many configurations: %d, "
		    "using maximum allowed: %d\n", ncfg, USB_MAXCONFIG);
		udev->descriptor.bNumConfigurations = ncfg = USB_MAXCONFIG;
	}

	if (ncfg < 1) {
		ERROR(dev, "no configurations\n");
		return -EINVAL;
	}

//	DBG(dev, "Previous config ptr=%p\n", udev->config);
	/* Free previous config, if any? */

	udev->config = kzalloc(ncfg * sizeof(struct usb_host_config), GFP_KERNEL);

	dev->udev = udev;

	INFO(dev, "Attaching!\n");

	status = usb_gadget_register_driver(&proxy_gadget_driver);
	if (status) {
		ERROR(dev, "usb_gadget_register_driver failed %d\n", status);
		return status;
	}

	return 0;
}

static void device_disconnect(struct usb_device *udev) {
	struct proxy_dev	*dev;
	int status;
	dev = &usb_proxy_gadget;

	INFO(dev, "device_disconnect\n");

	/* Kill urbs, if active */
	if (dev->ep1->urb)
		usb_kill_urb(dev->ep1->urb);

	if (dev->ep2->urb)
		usb_kill_urb(dev->ep2->urb);

	kfree(dev->ctrlreq);

	status = usb_gadget_unregister_driver(&proxy_gadget_driver);
	if (status)
		ERROR(dev, "usb_gadget_unregister_driver %d\n", status);

	memset(&proxy_gadget_driver, 0, sizeof(proxy_gadget_driver));
}

static int device_suspend(struct usb_device *udev, pm_message_t message) {
	struct proxy_dev	*dev;
	dev = &usb_proxy_gadget;

	INFO(dev, "device_suspend\n");

	return 0;
}

static int device_resume(struct usb_device *udev, pm_message_t message) {
	struct proxy_dev	*dev;
	dev = &usb_proxy_gadget;

	INFO(dev, "device_resume\n");

	return 0;
}

static struct usb_device_driver proxy_device_driver = {
	.name = "proxy",
	.probe = device_probe,
	.disconnect = device_disconnect,

	.suspend = device_suspend,
	.resume = device_resume,
	.supports_autosuspend = 0,
};

/*-------------------------------------------------------------------------*/

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Nicolas Boichat");
MODULE_LICENSE("GPL");

static int __init
init(void)
{
	int status;

	/* register this driver with the USB subsystem */
	status = usb_register_device_driver(&proxy_device_driver, THIS_MODULE);
	if (status) {
		ERROR(dev, "usb_register failed. Error number %d", status);
		return status;
	}

	return status;
}
module_init(init);

static void __exit
cleanup(void)
{
	usb_deregister_device_driver(&proxy_device_driver);
}
module_exit(cleanup);
