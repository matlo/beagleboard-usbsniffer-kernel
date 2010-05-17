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

#define DRIVER_DESC		"Proxy Gadget"
#define DRIVER_VERSION		"2010 ..."

static const char shortname [] = "gadgetproxy";
static const char driver_desc [] = DRIVER_DESC;

/*-------------------------------------------------------------------------*/

struct proxy_dev {
	spinlock_t		lock;		/* lock this structure (FIXME: useless for now) */
	struct usb_gadget	*gadget;
	struct usb_request	*req;		/* for control responses */

	/* 0 - nothing, 1 - connected, 2 - set_configuration sent, 3 - configured */
	/* TODO: Actually use that, + define some constants */
	u32                     device_state;
	struct usb_device	*udev;			/* the usb device for this device */

	/* EP1 stuff */
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

/* Some systems will want different product identifers published in the
 * device descriptor, either numbers or strings or both.  These string
 * parameters are in UTF-8 (superset of ASCII's 7 bit characters).
 */

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

/* USB DRIVER HOOKUP (to the hardware driver, below us), mostly
 * ep0 implementation:  descriptors, config management, setup().
 * also optional class-specific notification interrupt transfer.
 */

/*
 * DESCRIPTORS ... most are static, but strings and (full) configuration
 * descriptors are built on demand.
 */

#define STRING_MANUFACTURER		1
#define STRING_PRODUCT			2
#define STRING_SERIALNUM		3

/* holds our biggest descriptor */
#define USB_DESC_BUFSIZE		256
#define USB_BUFSIZE			8192

/* This device advertises one configuration. */
#define DEV_CONFIG_VALUE		1
#define	PROXY_INTERFACE		0

/*
Logitech mouse configuration

===
Bus 001 Device 005: ID 046d:c00e Logitech, Inc. M-BJ58/M-BJ69 Optical Wheel Mouse
Device Descriptor:
  bLength                18
  bDescriptorType         1
  bcdUSB               2.00
  bDeviceClass            0 (Defined at Interface level)
  bDeviceSubClass         0
  bDeviceProtocol         0
  bMaxPacketSize0         8
  idVendor           0x046d Logitech, Inc.
  idProduct          0xc00e M-BJ58/M-BJ69 Optical Wheel Mouse
  bcdDevice           11.10
  iManufacturer           1 Logitech
  iProduct                2 USB-PS/2 Optical Mouse
  iSerial                 0
  bNumConfigurations      1
  Configuration Descriptor:
    bLength                 9
    bDescriptorType         2
    wTotalLength           34
    bNumInterfaces          1
    bConfigurationValue     1
    iConfiguration          0
    bmAttributes         0xa0
      (Bus Powered)
      Remote Wakeup
    MaxPower               98mA
    Interface Descriptor:
      bLength                 9
      bDescriptorType         4
      bInterfaceNumber        0
      bAlternateSetting       0
      bNumEndpoints           1
      bInterfaceClass         3 Human Interface Device
      bInterfaceSubClass      1 Boot Interface Subclass
      bInterfaceProtocol      2 Mouse
      iInterface              0
        HID Device Descriptor:
          bLength                 9
          bDescriptorType        33
          bcdHID               1.10
          bCountryCode            0 Not supported
          bNumDescriptors         1
          bDescriptorType        34 Report
          wDescriptorLength      52
         Report Descriptors:
           ** UNAVAILABLE **
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x81  EP 1 IN
        bmAttributes            3
          Transfer Type            Interrupt
          Synch Type               None
          Usage Type               Data
        wMaxPacketSize     0x0004  1x 4 bytes
        bInterval              10
Device Status:     0x0000
  (Bus Powered)
*/

static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,
	.bcdUSB =		cpu_to_le16(0x0200),
	.bDeviceClass =		0,
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
	.bMaxPacketSize0 =      8,
	.idVendor =		cpu_to_le16(0x046d),
	.idProduct =		cpu_to_le16(0xc00e),
	.bcdDevice =            cpu_to_le16(11<<8 | 10),
	.iManufacturer =	STRING_MANUFACTURER,
	.iProduct =		STRING_PRODUCT,
	.iSerialNumber =	0,
	.bNumConfigurations =	1
};

static struct usb_config_descriptor config_desc = {
	.bLength =		sizeof config_desc,
	.bDescriptorType =	USB_DT_CONFIG,

	/* compute wTotalLength on the fly */
	.bNumInterfaces =	1,
	.bConfigurationValue =	1,
	.iConfiguration =	0,
	.bmAttributes =		0xa0,
	.bMaxPower =		98/2,
};

static struct usb_interface_descriptor intf_desc = {
	.bLength =		sizeof intf_desc,
	.bDescriptorType =	4,
	.bInterfaceNumber =	0,
	.bNumEndpoints =	1,
	.bInterfaceClass =	3,
	.bInterfaceSubClass =	1,
	.bInterfaceProtocol =	2,
	.iInterface =		0
};

/* HID descriptor missing here */
static char hid_desc[] = {
	0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 0x34, 0x00
};

static struct usb_endpoint_descriptor ep1_in_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	USB_DIR_IN,
	.bmAttributes =		USB_ENDPOINT_XFER_INT,
	.wMaxPacketSize =       0x0004,
	/* FIXME: Original value was 10, i.e. 10ms for LS/FS.
	 * But HS is measured in microframes (0.125ms),
	 * and polling interval is 2**(bInterval-1) => 2**(6-1)*0.125 = 4 ms.
	 * See USB specs p.299 (Table 9-13). */
	.bInterval =            7
};

static const struct usb_descriptor_header *proxy_function [] = {
	(struct usb_descriptor_header *) &intf_desc,
	(struct usb_descriptor_header *) &hid_desc,
	(struct usb_descriptor_header *) &ep1_in_desc,
	NULL
};

/*-------------------------------------------------------------------------*/

/* FIXME: Strings should be coming from the device. */

/* descriptors that are built on-demand */

static char				manufacturer [50];
static char				product_desc [40] = DRIVER_DESC;

/* static strings, in UTF-8 */
static struct usb_string		strings [] = {
	{ STRING_MANUFACTURER,	manufacturer, },
	{ STRING_PRODUCT,	product_desc, },
	{  }		/* end of list */
};

static struct usb_gadget_strings	stringtab = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings,
};

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

static int
config_buf(enum usb_device_speed speed, u8 *buf, u8 type, unsigned index)
{
	int					len;
	const struct usb_descriptor_header	**function;

	function = proxy_function;

	if (index >= device_desc.bNumConfigurations)
		return -EINVAL;

	len = usb_gadget_config_buf(&config_desc, buf, USB_DESC_BUFSIZE,
			function);
	if (len < 0)
		return len;
	((struct usb_config_descriptor *) buf)->bDescriptorType = type;
	return len;
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
	int			value = -EOPNOTSUPP;
	u16			wIndex = le16_to_cpu(ctrl->wIndex);
	u16			wValue = le16_to_cpu(ctrl->wValue);
	u16			wLength = le16_to_cpu(ctrl->wLength);

	struct urb *urb;

	unsigned int pipe;

	DBG(dev, "ctrl req%02x(%02x).%02x v%04x i%04x l%d\n",
	    ctrl->bRequestType, ctrl->bRequestType&USB_TYPE_MASK,
	    ctrl->bRequest, wValue, wIndex, wLength);

	req->complete = proxy_setup_complete;

	switch (ctrl->bRequestType&USB_TYPE_MASK) {

	case USB_TYPE_STANDARD:

		DBG(dev, "standard\n");

		switch (ctrl->bRequest) {

		case USB_REQ_GET_DESCRIPTOR:
			DBG(dev, "descriptor\n");

			if (ctrl->bRequestType != USB_DIR_IN)
				goto unknown;
			switch (wValue >> 8) {

			case USB_DT_DEVICE:
				DBG(dev, "DT_DEVICE\n");
				value = min(wLength, (u16) sizeof device_desc);
				memcpy(req->buf, &device_desc, value);
				break;
			case USB_DT_CONFIG:
				DBG(dev, "DT_CONFIG\n");
				value = config_buf(gadget->speed, req->buf,
						   wValue >> 8,
						   wValue & 0xff);
				if (value >= 0)
					value = min(wLength, (u16) value);
				break;

			case USB_DT_STRING:
				DBG(dev, "DT_STRING\n");
				value = usb_gadget_get_string(&stringtab,
						wValue & 0xff, req->buf);
				if (value >= 0)
					value = min(wLength, (u16) value);
				break;
			default:
				goto unknown;
			}
			break;

		case USB_REQ_SET_CONFIGURATION:
			if (ctrl->bRequestType != 0)
				break;
			if (gadget->a_hnp_support)
				DBG(dev, "HNP available\n");
			else if (gadget->a_alt_hnp_support)
				DBG(dev, "HNP needs a different root port\n");
			DBG(dev, "USB set configuration\n");
			value = 0;
			dev->device_state = 2;
			goto unknown;
			break;
		case USB_REQ_GET_CONFIGURATION:
			if (ctrl->bRequestType != USB_DIR_IN)
				break;
			//*(u8 *)req->buf = dev->config;
			value = min(wLength, (u16) 1);
			DBG(dev, "USB get configuration\n");
			goto unknown;
			break;

		case USB_REQ_SET_INTERFACE:
			DBG(dev, "USB set interface\n");
			goto unknown;
			break;

		case USB_REQ_GET_INTERFACE:
			DBG(dev, "USB get interface\n");
			goto unknown;
			break;

		default:
			DBG(dev, "unknown standard req\n");
			goto unknown;
		}
		break;

	case USB_TYPE_CLASS:
		DBG(dev, "type class\n");
		goto unknown;

	default:
unknown:
		DBG(dev,
			"unknown ctrl req%02x.%02x v%04x i%04x l%d\n",
			ctrl->bRequestType, ctrl->bRequest,
			wValue, wIndex, wLength);

		if (ctrl->bRequestType | USB_DIR_IN) {
			pipe = usb_rcvctrlpipe(dev->udev, 0);

			DBG(dev, "DIR_IN (%p %d)\n", req->buf, req->length);

			/* This would be nice, but cannot be called from interrupt context... */
			/*value = usb_control_msg(dev->udev, pipe,
						ctrl->bRequest, ctrl->bRequestType,
						ctrl->wValue, ctrl->wIndex,
						req->buf, req->length, 0);*/

			urb = usb_alloc_urb(0, GFP_NOIO);
			if (!urb)
				return -ENOMEM;

			/* FIXME: check buf and length (there could be some overruns) */
			/* FIXME: we may need to copy the data, or req->buf may be overridden
			 * if 2 control requests follow one another */
			usb_fill_control_urb(urb, dev->udev, pipe, (unsigned char *)ctrl,
					     req->buf, wLength,
					     device_setup_complete, gadget);
			
			usb_submit_urb(urb, GFP_ATOMIC);

			DBG(dev, "URB submitted...\n");
			return 0;

		} else {
			pipe = usb_sndctrlpipe(dev->udev, 0);

			DBG(dev, "DIR_OUT\n");

			/* TODO: Implement OUT control requests as well */
		}
		

		break;
	}

	/* If we reach here, it means that we can answer to the
	 * request on our own, without asking the device. 
	 * (this should be rare in the end) */

	/* respond with data transfer before status phase? */
	if (value >= 0) {
		req->length = value;
		req->zero = value < wLength;
		value = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
		DBG(dev, "ep_queue --> %d\n", value);
		if (value < 0) {
			req->status = 0;
			proxy_setup_complete(gadget->ep0, req);
		}
	}

	/* host either stalls (value < 0) or reports success */
	return value;
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

	usb_ep_fifo_flush(dev->gadget_ep1);
	usb_ep_disable(dev->gadget_ep1);

	set_gadget_data(gadget, NULL);
}

static int __init
proxy_bind(struct usb_gadget *gadget)
{
	struct proxy_dev	*dev;
	struct usb_ep		*in_ep;
	int			status = -ENOMEM;
//	int			gcnum;

	DBG(dev, "%s\n", __func__);

	dev = &usb_proxy_gadget;

	if (gadget_is_sa1100(gadget)) {
		/* hardware can't write zero length packets. */
		ERROR(dev, "SA1100 controller is unsupport by this driver\n");
		goto fail;
	}

#if 0
	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0) {
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	} else {
		dev_warn(&gadget->dev, "controller '%s' not recognized\n",
			gadget->name);
		/* unrecognized, but safe unless bulk is REALLY quirky */
		device_desc.bcdDevice =
			cpu_to_le16(0xFFFF);
	}
#endif
	snprintf(manufacturer, sizeof(manufacturer), "%s %s with %s",
		init_utsname()->sysname, init_utsname()->release,
		gadget->name);

	/* all we really need is bulk IN/OUT (FIXME: what does that mean?) */
	usb_ep_autoconfig_reset(gadget);
	in_ep = usb_ep_autoconfig(gadget, &ep1_in_desc);
	if (!in_ep) {
		dev_err(&gadget->dev, "can't autoconfigure on %s\n",
			gadget->name);
		return -ENODEV;
	}
	in_ep->driver_data = dev;	/* claim */

	spin_lock_init(&dev->lock);

	dev->gadget_ep1 = in_ep;

	/* preallocate control message data and buffer */
	dev->req = proxy_req_alloc(gadget->ep0, USB_DESC_BUFSIZE,
			GFP_KERNEL);
	if (!dev->req) {
		status = -ENOMEM;
		goto fail;
	}

	dev->req->complete = proxy_setup_complete;

	/* TODO: In theory, I should not update that field, but
	 * MUSB forces High-speed, which means 64 bytes.
	 * In Full-speed, we could support 8, 16, 32, 64 bytes
	 * (8 is what low-speed needs).
	 * See USB 2.0 specs, 5.5.3. */
	DBG(dev, "Desc maxpacketsize: %d - Gadget: %d\n",
	    device_desc.bMaxPacketSize0, gadget->ep0->maxpacket);
	device_desc.bMaxPacketSize0 = gadget->ep0->maxpacket;

	/* finish hookup to lower layer ... */
	dev->gadget = gadget;
	set_gadget_data(gadget, dev);
	gadget->ep0->driver_data = dev;

	INFO(dev, "%s, version: " DRIVER_VERSION "\n", driver_desc);
	INFO(dev, "using %s, IN %s\n", gadget->name, in_ep->name);

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

/* Callback, when a ctrl request has come back from the device */
static void device_setup_complete(struct urb *urb) {
	struct usb_gadget    *gadget = urb->context; 
	struct proxy_dev	*dev = get_gadget_data(gadget);
	struct usb_request	*req = dev->req;
	int			value = -EOPNOTSUPP;

	DBG(dev, "Device setup complete (%d; %d)!",
	    urb->status,
	    urb->actual_length);

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
	if (dev->device_state == 2) {
		/* The kernel refuses to send any non-control urbs if
		 * udev->state is not CONFIGURED */
		/* TODO: Maybe we should call the standard usb_set_configuration,
		 * to initialize the dev properly. */
		/* TODO: What about stuff like bandwidth allocation done
		 * in usb_set_configuration?!? */
		//usb_set_configuration(dev->udev, XXX);
		dev->udev->state = USB_STATE_CONFIGURED;

		/* Init EP1 on the gagdet side... */
		value = usb_ep_enable(dev->gadget_ep1, &ep1_in_desc);
		if (value)
			ERROR(dev, "can't enable %s, result %d\n", dev->gadget_ep1->name, value);

		/* ... and on the device side */
		device_connect_ep1(dev);

		dev->device_state = 3;
	}
}

/* EP1IN (interrupt) callback */
static void device_ep1_irq(struct urb *urb)
{
	struct proxy_dev *dev = urb->context;
	//struct usb_device *udev = dev->udev;
	int status;
	signed char *data = dev->device_ep1_data;
	struct usb_request	*req;
	signed char *gdata;

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

	/*DBG(dev, "Got an EP1 (%d; %x %d %d %d)!\n",
	  urb->actual_length, data[0], data[1], data[2], data[3]);*/

	/* allocate EP1 req */
	req = proxy_req_alloc(dev->gadget_ep1, urb->actual_length,
			GFP_KERNEL);
	if (!req) {
		ERROR(dev, "Cannot allocate request.\n");
		goto resubmit;
	}

	req->complete = proxy_ep1_complete;
	req->context = dev;
	gdata = req->buf;

	gdata[0] = data[0];
	gdata[1] = data[1];
	gdata[2] = data[2];
	gdata[3] = data[3];
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

	INFO(dev, "Connecting EP1...\n");

	dev->device_ep1_urb = usb_alloc_urb(0, GFP_KERNEL);
	/* TODO: Sanity check */

	dev->device_ep1_data = usb_buffer_alloc(udev, 4, GFP_ATOMIC, &dev->device_ep1_dma);
	/* TODO: Sanity check */

	pipe = usb_rcvintpipe(udev, ep1_in_desc.bEndpointAddress);

	/* Setup an urb on the device side for EP1 */
	usb_fill_int_urb(dev->device_ep1_urb, udev, pipe,
			 dev->device_ep1_data, 4 /* Check! */,
			 device_ep1_irq, dev, ep1_in_desc.bInterval);
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
	struct proxy_dev	*dev;
	dev = &usb_proxy_gadget;

	INFO(dev, "device_probe (%04x:%04x)\n",
	     le16_to_cpu(udev->descriptor.idVendor), le16_to_cpu(udev->descriptor.idProduct));

	if (dev->udev) {
		INFO(dev, "Already attached to another device!\n");
		return -1;
	}

	INFO(dev, "Attaching!\n");

	status = usb_gadget_register_driver(&proxy_gadget_driver);
	if (status) {
		DBG(dev, "usb_gadget_register_driver failed %d\n", status);
		return status;
	}

	dev->udev = udev;

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

	status = usb_gadget_unregister_driver(&proxy_gadget_driver);
	if (status)
		ERROR(dev, "usb_gadget_unregister_driver %d\n", status);

	dev->udev = NULL;
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
