/*
 * proxy.c -- Proxy gadget driver
 *
 * Copyright (C) 2010 Nicolas Boichat
 *
 * Based on Printer gadget:
 *
 * Copyright (C) 2003-2005 David Brownell
 * Copyright (C) 2006 Craig W. Nadler
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

struct proxy_dev {
	spinlock_t		lock;		/* lock this structure (FIXME: useless for now) */
	struct usb_gadget	*gadget;

	struct usb_device	*udev;			/* the usb device for this device */

 	/* EP0 stuff */
	struct usb_ctrlrequest  *ctrlreq;
	struct usb_request	*req;

	/* EP1 stuff */
	struct usb_endpoint_descriptor* ep1_in_desc;

	/* Gadget side */
	struct usb_ep		*gadget_ep1;

	/* Device side */
	signed char             *device_ep1_data;
	struct urb              *device_ep1_urb;
	dma_addr_t              device_ep1_dma;
};

static struct proxy_dev usb_proxy_gadget;

static int device_connect_ep1(struct proxy_dev	*dev);
static void device_setup_complete(struct urb *urb);

/*-------------------------------------------------------------------------*/

#define DEBUG 1

#define xprintk(d, level, fmt, args...) \
	printk(level "%s: " fmt, DRIVER_DESC, ## args)

#ifdef DEBUG
#define DBG(dev, fmt, args...) \
	xprintk(dev, KERN_DEBUG, fmt, ## args)
#else
#define DBG(dev, fmt, args...) \
	do { } while (0)
#endif /* DEBUG */

#ifdef VERBOSE
#define VDBG(dev, fmt, args...) \
	xprintk(dev, KERN_DEBUG, fmt, ## args)
#else
#define VDBG(dev, fmt, args...) \
	do { } while (0)
#endif /* VERBOSE */

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
	if (req->status || req->actual != req->length)
		DBG((struct proxy_dev *) ep->driver_data,
				"setup complete --> %d, %d/%d\n",
				req->status, req->actual, req->length);
}

static void proxy_ep1_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct proxy_dev * dev = ep->driver_data;

/*
	DBG(dev,
	    "EP1 complete --> %d, %d/%d\n",
				req->status, req->actual, req->length);
*/

	//dev->gadget_ep1_busy = 0;

	proxy_req_free(dev->gadget_ep1, req);
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

	DBG(dev, "ctrl req%02x(%02x).%02x v%04x i%04x l%d\n",
	    ctrl->bRequestType, ctrl->bRequestType&USB_TYPE_MASK,
	    ctrl->bRequest, wValue, wIndex, wLength);

	req->complete = proxy_setup_complete;

	if (ctrl->bRequestType & USB_DIR_IN) {
		pipe = usb_rcvctrlpipe(dev->udev, 0);
		
		DBG(dev, "DIR_IN (%p %d)\n", req->buf, req->length);
	} else {
		pipe = usb_sndctrlpipe(dev->udev, 0);
		
		DBG(dev, "DIR_OUT\n");
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
	
	DBG(dev, "URB submitted (%d)...\n", value);
	return 0;
}

static void
proxy_disconnect(struct usb_gadget *gadget)
{
	struct proxy_dev	*dev = get_gadget_data(gadget);
	unsigned long		flags;

	DBG(dev, "%s\n", __func__);

	spin_lock_irqsave(&dev->lock, flags);

	/* TODO: Do something! */

	spin_unlock_irqrestore(&dev->lock, flags);
}

static void
proxy_unbind(struct usb_gadget *gadget)
{
	struct proxy_dev	*dev = get_gadget_data(gadget);


	DBG(dev, "%s\n", __func__);

	if (dev->req) {
		proxy_req_free(gadget->ep0, dev->req);
		dev->req = NULL;
	}

	/* FIXME: Ideally, we should dequeue reqs... */

	if (dev->gadget_ep1) {
		usb_ep_fifo_flush(dev->gadget_ep1);
		usb_ep_disable(dev->gadget_ep1);
	}

	set_gadget_data(gadget, NULL);
}

static int __init
proxy_bind(struct usb_gadget *gadget)
{
	struct proxy_dev	*dev;
	int			status = -ENOMEM;

	DBG(dev, "%s\n", __func__);

	dev = &usb_proxy_gadget;

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
	.speed		= USB_SPEED_FULL,

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

	DBG(dev, "Rewriting configuration...\n");

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

		DBG(dev, "Descriptor type %x\n", header->bDescriptorType);

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
				DBG(dev, "Updated bInterval %d->%d\n", d->bInterval, ninterval);
				d->bInterval = ninterval;
			}
		}

		buffer += header->bLength;
	}
}

static void bridge_endpoints(struct usb_gadget *gadget, int index) {
	struct proxy_dev	*dev = get_gadget_data(gadget);
	int value;
	int i, j, k;
	struct usb_host_config* config;
	struct usb_host_interface *alt = NULL;

	DBG(dev, "Bridging endpoints.\n");

	for (i = 0; i < dev->udev->descriptor.bNumConfigurations; i++) {
		config = &dev->udev->config[i];
		DBG(dev, "Config %d (%d)\n",
		    i, config->desc.bConfigurationValue);
		if (config->desc.bConfigurationValue == index) {
			DBG(dev, "Got the config!\n");
			for (j = 0; j < config->desc.bNumInterfaces; j++) {
				/* Set up endpoints for alternate interface setting 0 */
				alt = usb_find_alt_setting(config, i, 0);
				if (!alt)
					/* No alt setting 0? Pick the first setting. */
					alt = &config->intf_cache[i]->altsetting[0];

				DBG(dev, "Interface %d: %d endpoints\n", j, alt->desc.bNumEndpoints);

				for (k = 0; k < alt->desc.bNumEndpoints; k++) {
					DBG(dev, "Endpoint %d: %02x\n",
					    k, alt->endpoint[k].desc.bEndpointAddress);
					if (alt->endpoint[k].desc.bEndpointAddress == 0x81) {
						dev->ep1_in_desc = &alt->endpoint[k].desc;
					}
					
				}
			}
			break;
		}
	}

	if (!dev->ep1_in_desc) {
		ERROR(dev, "Cannot find EP1!\n");
		return;
	}

	/* Init EP1 on the gagdet side... */
	/* Note: usb_ep_autoconfig does not look at bInterval,
	 * so it should be ok to use the non-rewritten descriptor. */
	dev->gadget_ep1 = usb_ep_autoconfig(gadget, dev->ep1_in_desc);
	if (!dev->gadget_ep1) {
		dev_err(&gadget->dev, "can't autoconfigure on %s\n",
			gadget->name);
		return;
	}
	dev->gadget_ep1->driver_data = dev;	/* claim */
	
	value = usb_ep_enable(dev->gadget_ep1, dev->ep1_in_desc);
	if (value)
		ERROR(dev, "can't enable %s, result %d\n", dev->gadget_ep1->name, value);
	
	/* ... and on the device side */
	device_connect_ep1(dev);
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

	DBG(dev, "Device setup complete (%d; %d)!",
	    urb->status,
	    urb->actual_length);

	DBG(dev, "ctrl req%02x(%02x).%02x v%04x\n",
	    ctrl->bRequestType, ctrl->bRequestType&USB_TYPE_MASK,
	    ctrl->bRequest, wValue);

	if ((ctrl->bRequestType == USB_DIR_IN) &&
	    (ctrl->bRequest == USB_REQ_GET_DESCRIPTOR)) {
		DBG(dev, "GET_DESCRIPTOR (%04x)\n", wValue);
		switch (wValue >> 8) {
		case USB_DT_DEVICE:
			DBG(dev, "DT_DEVICE\n");
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
			DBG(dev, "DT_CONFIG (%d)\n", index);
			if (index >= dev->udev->descriptor.bNumConfigurations) {
				ERROR(dev, "Config %d, while only %d were expected!\n",
				      index, dev->udev->descriptor.bNumConfigurations);
				break;
			}

			/* Kernel 2.6.35 needs dev->udev, 2.6.32 &dev->udev->dev */
			value = usb_parse_configuration(dev->udev, index,
				   &dev->udev->config[index], req->buf,
				   urb->actual_length);
			if (value < 0) {
				DBG(dev, "Parse configuration error\n");
			}
			rewrite_config(req->buf, urb->actual_length);
			break;
		}
	}

	/* TODO: Not sure what to do if urb->status < 0... */
	req->length = urb->actual_length;
	req->zero = 0; //value < wLength;
	value = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
	DBG(dev, "ep_queue --> %d\n", urb->status);
	if (value < 0) {
		req->status = 0;
		proxy_setup_complete(gadget->ep0, req);
	}

	/* If SET_CONFIGURATION has been sent to the host,
	 * initialize the other endpoints. */
	/* TODO: Handle multiple configurations */
	if (((ctrl->bRequestType&USB_TYPE_MASK) == USB_TYPE_STANDARD) &&
	    (ctrl->bRequest == USB_REQ_SET_CONFIGURATION) ) {
		DBG(dev, "SET_CONFIGURATION\n");

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
	struct usb_request	*req;

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

	//DBG(dev, "Got an EP1 urb back (%d)!\n", urb->actual_length);

	/* allocate EP1 req */
	req = proxy_req_alloc(dev->gadget_ep1, urb->actual_length,
			GFP_KERNEL);
	if (!req) {
		ERROR(dev, "Cannot allocate request.\n");
		goto resubmit;
	}

	req->complete = proxy_ep1_complete;
	req->context = dev;

	memcpy(req->buf, dev->device_ep1_data, urb->actual_length);

	req->length = urb->actual_length;
	req->zero = 0;
	
	status = usb_ep_queue(dev->gadget_ep1, req, GFP_ATOMIC);
	//DBG(dev, "ep_queue --> %d\n", status);
	if (status < 0) {
		req->status = 0;
		proxy_ep1_complete(dev->gadget_ep1, req);
	}

resubmit:
	status = usb_submit_urb (urb, GFP_ATOMIC);
	if (status)
		ERROR(dev, "can't resubmit intr, status %d\n", status);
}

/* Connect EP1 on the device side */
static int device_connect_ep1(struct proxy_dev	*dev) {
	struct usb_device *udev = dev->udev;
	int pipe;
	int status;
	u16 wMaxPacketSize = le16_to_cpu(dev->ep1_in_desc->wMaxPacketSize);

	INFO(dev, "Connecting EP1...\n");

	dev->device_ep1_urb = usb_alloc_urb(0, GFP_KERNEL);
	/* TODO: Sanity check */

#if 0
	dev->device_ep1_data = usb_buffer_alloc(udev, wMaxPacketSize, GFP_ATOMIC, &dev->device_ep1_dma);
#else
	dev->device_ep1_data = usb_alloc_coherent(udev, wMaxPacketSize, GFP_ATOMIC, &dev->device_ep1_dma);
#endif
	/* TODO: Sanity check */
	/* FIXME: this buffer is never freed! */

	pipe = usb_rcvintpipe(udev, dev->ep1_in_desc->bEndpointAddress);

	/* Setup an urb on the device side for EP1 */
	usb_fill_int_urb(dev->device_ep1_urb, udev, pipe,
			 dev->device_ep1_data, wMaxPacketSize,
			 device_ep1_irq, dev, dev->ep1_in_desc->bInterval);
	dev->device_ep1_urb->transfer_dma = dev->device_ep1_dma;
	dev->device_ep1_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	status = usb_submit_urb(dev->device_ep1_urb, GFP_ATOMIC);

	if (status) {
		ERROR(dev, "can't submit EP1 urb, status %d\n", status);
		/* TODO: Do something... */
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

	DBG(dev, "Previous config ptr=%p\n", udev->config);

	udev->config = kzalloc(ncfg * sizeof(struct usb_host_config), GFP_KERNEL);

	dev->udev = udev;

	INFO(dev, "Attaching!\n");

	status = usb_gadget_register_driver(&proxy_gadget_driver);
	if (status) {
		DBG(dev, "usb_gadget_register_driver failed %d\n", status);
		return status;
	}

	return 0;
}

static void device_disconnect(struct usb_device *udev) {
	struct proxy_dev	*dev;
	int status;
	dev = &usb_proxy_gadget;

	INFO(dev, "device_disconnect\n");

	/* Kill EP1 urb, if active */
	if (dev->device_ep1_urb)
		usb_kill_urb(dev->device_ep1_urb);

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
