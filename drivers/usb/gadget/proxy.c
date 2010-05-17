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
/* 
 * Endpoint traffic goes the following way:
 * 
 * Endpoint 0:
 *   - gadget_setup receives a control request (Note: this is asynchronous,
 *     that is, the host may not wait for the previous request's reply
 *     before sending a new one, and I have not found a way to prevent
 *     the gadget controller from sending a new packet)
 *   - If no other control request is flying, process it in
 *     gadget_handle_ctrlrequest, else queue.
 *   - gadget_handle_ctrlrequest, 2 possibilities here:
 *     a) IN request, or OUT request without data (wLength = 0)
 *        - Copy the control request to the URB, submit it.
 *        - Callback: device_setup_complete: process the packet (analyse
 *          the descriptor, take action on standard requests, etc...)
 *        - Submit a reply request.
 *        - Callback: gadget_setup_complete: if another control request
 *          is waiting process it (gadget_handle_ctrlrequest).
 *     b) OUT request, with data (wLength > 0), note: this is never
 *        a standard request (well, it could be a SET_DESCRIPTOR,
 *        wondering if any device uses that).
 *        - Submit a request to read the rest of the data.
 *        - Callback: gadget_setup_out_complete: copy the control request
 *          + the data, and submit the URB.
 *        - Callback: device_setup_out_complete: if another control request
 *          is waiting process it (gadget_handle_ctrlrequest).
 *
 * Any other IN endpoint:
 *   - bridge_endpoint: setups endpoints, and submit the URB.
 *   - Callback: device_epin_irq, copy the data to the request, submit it.
 *   - Callback: gadget_epin_complete, resubmit the URB
 *
 * Any other OUT endpoint:
 *   - bridge_endpoint: setups endpoints, and submit the request.
 *   - Callback: gadget_epout_complete, copy the data to the URB, submit it.
 *   - Callback: device_epout_irq, resubmit the request.
 */


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/moduleparam.h>

#include <asm/byteorder.h>

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
/* TODO: epautoconfig is not used a lot anymore... */
#include "epautoconf.c"

/*-------------------------------------------------------------------------*/
/* Module stuff */

#define DRIVER_DESC		"Proxy Gadget"
#define DRIVER_VERSION		"2010 ..."

static const char shortname[]   = "gadgetproxy";
static const char driver_desc[] = DRIVER_DESC;

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Nicolas Boichat");
MODULE_LICENSE("GPL");

static int debug = 0;
module_param(debug, uint, 1);
MODULE_PARM_DESC(debug, "Debug level, default=0");

/* --- */

#define xprintk(level, fmt, args...) \
	printk(level "%s: " fmt, DRIVER_DESC, ## args)

#define DBG(level, fmt, args...)					\
	do {								\
		if (debug >= level) xprintk(KERN_DEBUG, fmt, ## args);	\
	} while (0)

#define ERROR(fmt, args...) \
	xprintk(KERN_ERR, "ERROR: " fmt, ## args)
#define WARNING(fmt, args...) \
	xprintk(KERN_WARNING, fmt, ## args)
#define INFO(fmt, args...)			\
	xprintk(KERN_INFO, fmt, ## args)

/*-------------------------------------------------------------------------*/

#define USB_MAXCONFIG                  8      /* Arbitrary limit */
#define USB_MAXENDPOINT               32      /* 15 IN, 15 OUT */
#define USB_DESC_BUFSIZE             512

/*-------------------------------------------------------------------------*/

struct proxy_endpoint {
	struct usb_endpoint_descriptor* desc;

	/* Used for ep0 only */
	struct usb_ctrlrequest	*ctrlreq;
	/* Buffer for a second ctrlreq, to be sent later */
	/* FIXME: I don't really like that, I would prefer having multiple
	 * sets of req/urb (that can be used for other EPs as well). */
	struct usb_ctrlrequest	*ctrlreq_pending;
	int			flying;

	/* Gadget side */
	struct usb_ep		*gadget_ep;
	struct usb_request	*req;

	/* Device side */
	struct usb_host_endpoint *device_ep;
	struct urb		*urb;
};

struct proxy_dev {
	spinlock_t		lock;	  /* lock this structure (FIXME: useless for now) */
	struct usb_gadget	*gadget;

	struct usb_device	*udev;	  /* the usb device for this device */

	struct proxy_endpoint   ep0;

	/* Other EPs stuff */
	struct proxy_endpoint	*eps[USB_MAXENDPOINT];
};

/*-------------------------------------------------------------------------*/

static struct proxy_dev usb_proxy_gadget;

static int device_setup_ep(struct proxy_dev *dev, struct proxy_endpoint* ep);
static void device_setup_complete(struct urb *urb);
static void device_setup_out_complete(struct urb *urb);
static void device_epout_irq(struct urb *urb);
static void device_free_ep(struct proxy_dev *dev, struct proxy_endpoint* ep);

static int gadget_handle_ctrlrequest(struct usb_gadget *gadget,
				const struct usb_ctrlrequest *ctrl);

/*-------------------------------------------------------------------------*/
/* Gadget side of the driver */

/* Allocate and free usb_request (gadget equivalent of urbs) */
static struct usb_request *
gadget_req_alloc(struct usb_ep *ep, unsigned len, gfp_t gfp_flags)
{
	struct usb_request *req;

	req = usb_ep_alloc_request(ep, gfp_flags);

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
gadget_req_free(struct usb_ep *ep, struct usb_request *req)
{
	if (ep != NULL && req != NULL) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

/* EP operations callbacks */

/* EP0 IN or 0-length OUT callback. */
static void gadget_setup_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct proxy_dev *dev = &usb_proxy_gadget;

	DBG(20, "setup complete --> %d, %d/%d\n",
	    req->status, req->actual, req->length);

	/* Handle the next ctrl request, if any was buffered. */
	dev->ep0.flying -= 1;
	if (dev->ep0.flying > 0)
		gadget_handle_ctrlrequest(dev->gadget, dev->ep0.ctrlreq_pending);
}

/* EP0 OUT with data callback, send the data the we just read to the device */
static void gadget_setup_out_complete(struct usb_ep *ep, struct usb_request *req)
{
	int status, pipe;
	struct proxy_dev *dev = ep->driver_data;

	DBG(20,
	    "setup out complete --> %d, %d/%d\n",
	    req->status, req->actual, req->length);
	
	pipe = usb_sndctrlpipe(dev->udev, 0);
		
	/* FIXME: check buf and length (there could be some overruns) */
	usb_fill_control_urb(dev->ep0.urb, dev->udev, pipe, (void*)dev->ep0.ctrlreq,
			     req->buf, req->actual,
			     device_setup_out_complete, dev);
	/* TODO: Understand how the DMA works... */
	//urb->setup_dma = usbhid->cr_dma;
		
	status = usb_submit_urb(dev->ep0.urb, GFP_ATOMIC);
	DBG(20, "EP0 OUT URB submitted (%d)...\n", status);
	if (status) {
		ERROR("can't submit EP0 OUT URB, status %d\n", status);
		/* TODO: Can we do something here? */
	}
}


/* Gadget IN endpoint. Data was sent to the host,
 * resubmit the urb to the device. */
static void gadget_epin_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct proxy_endpoint *p_ep = req->context;
	int status;

	switch (req->status) {
	case 0:			/* success */
		break;
	case -ECONNRESET:	/* unlink */
	case -ENOENT:
	case -ESHUTDOWN:
		ERROR("EP-IN error: status %d, not resubmitting\n",
			req->status);
		return;
	/* -EPIPE:  should clear the halt (FIXME: Was does that mean?) */
	default:		/* error */
		ERROR("EP-IN error: status %d, resubmitting\n", req->status);
	}

	DBG(30, "EP-IN complete --> %d, %d/%d\n",
	    req->status, req->actual, req->length);

	status = usb_submit_urb(p_ep->urb, GFP_ATOMIC);
	if (status) {
		ERROR("can't resubmit URB, status %d\n", status);
		/* TODO: Can we do something here? */
	}
}

/* Gadget OUT endpoint. Data came from the host, forward to the device */
static void gadget_epout_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct proxy_endpoint *p_ep = req->context;
	int status;

	DBG(25, "EP-OUT complete --> %d, %d/%d\n",
	    req->status, req->actual, req->length);
	
	switch (req->status) {
	case 0:			/* success */
		break;
	case -ECONNRESET:	/* unlink */
	case -ENOENT:
	case -ESHUTDOWN:
		ERROR("EP-OUT error: status %d, not resubmitting\n",
			req->status);
		return;
	/* -EPIPE:  should clear the halt (FIXME: Was does that mean?) */
	default:		/* error */
		ERROR("EP-OUT error: status %d, resubmitting\n", req->status);
		goto resubmit;
	}

	memcpy(p_ep->urb->transfer_buffer, req->buf, req->actual);
	p_ep->urb->transfer_buffer_length = req->actual;

	status = usb_submit_urb(p_ep->urb, GFP_ATOMIC);
	DBG(25, "EP-OUT submit urb --> %d\n", status);
	if (status < 0) {
		ERROR("gadget_epout_complete submit urb --> %d\n", status);
		/* Bail out... */
	}

	return;

resubmit:
	status = usb_ep_queue(p_ep->gadget_ep, req, GFP_ATOMIC);
	DBG(25, "gadget_epout_complete ep_queue --> %d\n", status);
	if (status < 0) {
		ERROR("gadget_epout_complete ep_queue --> %d\n", status);
		/* Bail out... */
	}
}


/*-------------------------------------------------------------------------*/

/*
 * The setup() callback implements all the ep0 functionality that's not
 * handled lower down.
 * 
 * IN request: Submit URB request to the device, device_setup_complete
 * will submit the request back
 * 
 * OUT request: If wLength > 0, first submit a request to get the
 * remainder of the data, gadget_setup_out_complete will submit the URB.
 */
static int
gadget_setup(struct usb_gadget *gadget, const struct usb_ctrlrequest *ctrl)
{
	struct proxy_dev	*dev = get_gadget_data(gadget);
	u16			wIndex = le16_to_cpu(ctrl->wIndex);
	u16			wValue = le16_to_cpu(ctrl->wValue);
	u16			wLength = le16_to_cpu(ctrl->wLength);

	DBG(15, "ctrl req%02x(%02x).%02x v%04x i%04x l%d\n",
	    ctrl->bRequestType, ctrl->bRequestType&USB_TYPE_MASK,
	    ctrl->bRequest, wValue, wIndex, wLength);

	if (dev->ep0.flying >= 2) {
		ERROR(">=2 EP0 req flying (%d)! Ignoring...\n",
			dev->ep0.flying);
		return 0;
	} else if (dev->ep0.flying >= 1) {
		DBG(15, "1 EP0 req flying! (%d)\n", dev->ep0.flying);

		DBG(20, "Current: ctrl req%02x(%02x).%02x v%04x i%04x l%d\n",
		    ctrl->bRequestType, ctrl->bRequestType&USB_TYPE_MASK,
		    ctrl->bRequest, wValue, wIndex, wLength);

		wIndex = le16_to_cpu(dev->ep0.ctrlreq->wIndex);
		wValue = le16_to_cpu(dev->ep0.ctrlreq->wValue);
		wLength = le16_to_cpu(dev->ep0.ctrlreq->wLength);

		DBG(20, "Flying: ctrl req%02x(%02x).%02x v%04x i%04x l%d\n",
			dev->ep0.ctrlreq->bRequestType,
			dev->ep0.ctrlreq->bRequestType&USB_TYPE_MASK,
			dev->ep0.ctrlreq->bRequest, wValue, wIndex, wLength);

		memcpy(dev->ep0.ctrlreq_pending, ctrl, sizeof(*dev->ep0.ctrlreq_pending));

		dev->ep0.flying += 1;

		return 0;
	} else {
		dev->ep0.flying += 1;

		/* Nothing flying, handle it now */
		return gadget_handle_ctrlrequest(gadget, ctrl);
	}
}

/*
 * Handle a control request. Either it just arrived, or it was queued (because
 * another request was flying)
 */
static int
gadget_handle_ctrlrequest(struct usb_gadget *gadget,
			 const struct usb_ctrlrequest *ctrl)
{
	struct proxy_dev	*dev = get_gadget_data(gadget);
	struct usb_request	*req = dev->ep0.req;
	int			status;
	u16			wIndex = le16_to_cpu(ctrl->wIndex);
	u16			wValue = le16_to_cpu(ctrl->wValue);
	u16			wLength = le16_to_cpu(ctrl->wLength);

	unsigned int pipe;	

	DBG(15, "handling ctrl req%02x(%02x).%02x v%04x i%04x l%d\n",
	    ctrl->bRequestType, ctrl->bRequestType&USB_TYPE_MASK,
	    ctrl->bRequest, wValue, wIndex, wLength);

	/* ctrl is allocated on the stack in MUSB,
	   so copy it beforehand. */
	memcpy(dev->ep0.ctrlreq, ctrl, sizeof(*dev->ep0.ctrlreq));

	if (ctrl->bRequestType & USB_DIR_IN) {
		DBG(20, "DIR_IN\n");
		pipe = usb_rcvctrlpipe(dev->udev, 0);
	} else if (wLength == 0) {
		DBG(20, "DIR_OUT\n");
		pipe = usb_sndctrlpipe(dev->udev, 0);
	} else {
		DBG(20, "DIR_OUT, with data (%d)\n", wLength);
		
		req->complete = gadget_setup_out_complete;
		req->length = wLength;
		req->zero = 0;
		status = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
		DBG(20, "proxy_setup ep_queue --> %d\n", status);
		if (status < 0) {
			ERROR("DIR_OUT w/ data, proxy_setup ep_queue --> %d\n", status);
			/* Bail out... */
			return status;
		}

		return 0;
	}
	
	/* FIXME: check buf and wLength (there could be some overruns) */
	usb_fill_control_urb(dev->ep0.urb, dev->udev, pipe,
			(void*)dev->ep0.ctrlreq, req->buf, wLength,
			device_setup_complete, gadget);
	/* TODO: Understand how the DMA works... */
	//urb->setup_dma = usbhid->cr_dma;
		
	status = usb_submit_urb(dev->ep0.urb, GFP_ATOMIC);
	DBG(20, "URB submitted (%d)...\n", status);
	if (status < 0) {
		ERROR("proxy_handle_ctrlrequest submit urb --> %d\n", status);
		return status;
	}

	return 0;
}

static void
gadget_disconnect(struct usb_gadget *gadget)
{
	struct proxy_dev	*dev = get_gadget_data(gadget);
	unsigned long		flags;

	DBG(5, "%s\n", __func__);

	spin_lock_irqsave(&dev->lock, flags);

	/* TODO: Do something! */

	spin_unlock_irqrestore(&dev->lock, flags);
}

static void
gadget_unbind(struct usb_gadget *gadget)
{
	struct proxy_dev	*dev = get_gadget_data(gadget);
	int i;

	DBG(5, "%s\n", __func__);

	if (dev->ep0.req) {
		gadget_req_free(gadget->ep0, dev->ep0.req);
		dev->ep0.req = NULL;
	}

	/* TODO: Ideally, we should dequeue reqs, or does it come for
	 * free when we flush the FIFOs? */

	for (i = 0; i < USB_MAXENDPOINT; i++) {
		if (dev->eps[i]) {
			usb_ep_fifo_flush(dev->eps[i]->gadget_ep);
			usb_ep_disable(dev->eps[i]->gadget_ep);
			gadget_req_free(dev->eps[i]->gadget_ep, dev->eps[i]->req);
			kfree(dev->eps[i]);
		}
		dev->eps[i] = NULL;
	}

	set_gadget_data(gadget, NULL);
}

static int __init
gadget_bind(struct usb_gadget *gadget)
{
	struct proxy_dev	*dev= &usb_proxy_gadget;
	int			status = -ENOMEM;

	DBG(5, "%s\n", __func__);

	/* TODO: This should be more general, or warning if !MUSB */
	if (gadget_is_sa1100(gadget)) {
		/* hardware can't write zero length packets. */
		ERROR("SA1100 controller is unsupport by this driver\n");
		goto fail;
	}

	/* Reset host EPs. */
	usb_ep_autoconfig_reset(gadget);

	spin_lock_init(&dev->lock);

	/* preallocate control message data and buffer */
	dev->ep0.req = gadget_req_alloc(gadget->ep0, USB_DESC_BUFSIZE,
			GFP_KERNEL);
	if (!dev->ep0.req) {
		status = -ENOMEM;
		goto fail;
	}

	/* finish hookup to lower layer ... */
	dev->gadget = gadget;
	set_gadget_data(gadget, dev);
	gadget->ep0->driver_data = dev;

	INFO("using %s\n", gadget->name);

	return 0;

fail:
	gadget_unbind(gadget);
	return status;
}

/*---*/

static struct usb_gadget_driver proxy_gadget_driver = {
	.function	= (char *) driver_desc,
	.bind		= gadget_bind,
	.unbind		= gadget_unbind,

	.setup		= gadget_setup,
	.disconnect	= gadget_disconnect,

	.driver		= {
		.name		= (char *) shortname,
		.owner		= THIS_MODULE,
	},
};

/*-------------------------------------------------------------------------*/
/* USB gadget/device driver bridging part. */

/*
 * Find an appropriate gadget endpoint for the given description.
 * usb_ep_autoconfig does a similar job, but modify the endpoint address,
 * which is not acceptable for us.
 */
static struct usb_ep *find_gadget_endpoint(struct usb_gadget *gadget, 
					   struct usb_endpoint_descriptor* desc)
{
	struct usb_ep *ep = NULL;

	char myname[16];

	/* Note: This code is horrible, but it seems
	 * to be the way the gadget framework works. */
	snprintf(myname, 16, "ep%d%s",
		 desc->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK,
		 ((desc->bEndpointAddress & USB_ENDPOINT_DIR_MASK)
			 == USB_DIR_IN) ? "in": "out");

	DBG(15, "autoconf: Looking for endpoint %s\n", myname);

	list_for_each_entry (ep, &gadget->ep_list, ep_list) {
		if (!strcmp(ep->name, myname)) {
			return ep;
		}
	}

	return NULL;
}

static int gadget_setup_ep(struct usb_gadget *gadget, struct proxy_endpoint* ep)
{
	struct proxy_dev *dev = &usb_proxy_gadget;
	u16 wMaxPacketSize = le16_to_cpu(ep->desc->wMaxPacketSize);
	int status;

	ep->gadget_ep = find_gadget_endpoint(gadget, ep->desc);
	if (!ep->gadget_ep) {
		dev_err(&gadget->dev, "can't autoconfigure on %s\n",
			gadget->name);
		return -ENOENT;
	}
	ep->gadget_ep->driver_data = dev;	/* claim */
	
	status = usb_ep_enable(ep->gadget_ep, ep->desc);
	if (status) {
		ERROR("can't enable %s, result %d\n",
			ep->gadget_ep->name, status);
		return status;
	}
	
	ep->req = gadget_req_alloc(ep->gadget_ep, wMaxPacketSize,
					 GFP_ATOMIC);
	if (!ep->req) {
		ERROR("Cannot allocate request.\n");
		return -ENOMEM;
	}
	
	ep->req->complete = usb_endpoint_dir_in(ep->desc) ?
		gadget_epin_complete : gadget_epout_complete;
	ep->req->context = ep;

	ep->req->length = wMaxPacketSize;
	ep->req->zero = 0;

	return 0;
}

static void gadget_free_ep(struct proxy_endpoint* ep)
{
	usb_ep_fifo_flush(ep->gadget_ep);
	usb_ep_disable(ep->gadget_ep);
	gadget_req_free(ep->gadget_ep, ep->req);
	ep->req = NULL;
}

static void rewrite_config(unsigned char* buffer, int length) {
	struct proxy_dev *dev = &usb_proxy_gadget;
	struct usb_descriptor_header *header;
	struct usb_endpoint_descriptor *d;
	int ninterval;

	DBG(10, "Rewriting configuration...\n");

	while (length > 0) {
		if (length < sizeof(struct usb_descriptor_header)) {
			WARNING("Some excess bytes at the end of"
				" config descriptor?!?\n");
			break;
		}

		header = (struct usb_descriptor_header *) buffer;

		length -= header->bLength;

		if (length < 0) {
			WARNING("Incomplete config descriptor?!?\n");
			break;
		}

		DBG(15, "Descriptor type %x\n", header->bDescriptorType);

		if (header->bDescriptorType == USB_DT_ENDPOINT) {
			d = (struct usb_endpoint_descriptor *) buffer;

			/* Fixes bInterval for LS devices (measured in ms).
			 * But HS is measured in microframes (0.125ms),
			 * and polling interval is 2**(bInterval-1) =>
			 * 2**(7-1)*0.125 ms = 8 ms.
			 * For FS: 2**(4-1)*1 ms = 8 ms
			 * See USB specs p.299 (Table 9-13).
			 */

			if (dev->udev->speed == USB_SPEED_LOW) {
				/* TODO: FS/HS doesn't seem to change
				 * anything on my PC, and everything is still
				 * defined in microframes (1/8 ms)*/
				ninterval = ilog2(d->bInterval*8)+1;
				DBG(15, "Updated bInterval %d->%d\n",
					d->bInterval, ninterval);
				d->bInterval = ninterval;
			}
		}

		buffer += header->bLength;
	}
}

static int bridge_endpoint(struct usb_gadget *gadget,
			   struct usb_host_endpoint *ep) {
	struct usb_endpoint_descriptor* desc = &ep->desc;
	struct proxy_dev *dev = get_gadget_data(gadget);
	int n;
	int status;

	DBG(1, "Endpoint: %02x (%s %s), maxPacket=%d\n",
	    desc->bEndpointAddress,
	    usb_endpoint_dir_in(desc) ? "IN": "OUT",
	    usb_endpoint_xfer_bulk(desc) ? "bulk" :
	    usb_endpoint_xfer_control(desc) ? "control" :
	    usb_endpoint_xfer_int(desc) ? "int" :
	    usb_endpoint_xfer_isoc(desc) ? "isoc" :
	    "???",
	    le16_to_cpu(desc->wMaxPacketSize));
	
	for (n = 0; n < USB_MAXENDPOINT; n++) {
		if (!dev->eps[n])
			break;
	}

	if (n == USB_MAXENDPOINT) {
		ERROR("Too many endpoints!\n");
		return -EINVAL;
	}

	dev->eps[n] = kzalloc(sizeof(struct proxy_endpoint), GFP_ATOMIC);
	if (!dev->eps[n])
		return -ENOMEM;

	dev->eps[n]->device_ep = ep;
	dev->eps[n]->desc = desc;

	/* Init EP on the gagdet side... */
	status = gadget_setup_ep(gadget, dev->eps[n]);
	if (status < 0) {
		ERROR("cannot setup EP on the gadget side (%d).\n", status);
		goto error_gadget_setup;
	}

	/* ... and on the device side */
	status = device_setup_ep(dev, dev->eps[n]);
	if (status < 0) {
		ERROR("cannot setup EP on the device side (%d).\n", status);
		goto error_device_setup;
	}

	/* For IN endpoint, submit the urb, so that we can receive transfers. */
	if (usb_endpoint_dir_in(dev->eps[n]->desc)) {
		status = usb_submit_urb(dev->eps[n]->urb, GFP_ATOMIC);

		if (status < 0) {
			ERROR("bridge_endpoint: can't submit EP urb, status %d\n", status);
			/* bail out */
			goto error_submit;
		}
	} else { /* For OUT endpoint, submit the request. */
		status = usb_ep_queue(dev->eps[n]->gadget_ep, dev->eps[n]->req, GFP_ATOMIC);

		if (status < 0) {
			ERROR("bridge_endpoint: ep_queue --> status %d\n", status);
			/* bail out */
			goto error_submit;
		}
	}

	return 0;

error_submit:
	device_free_ep(dev, dev->eps[n]);

error_device_setup:
	gadget_free_ep(dev->eps[n]);
	
error_gadget_setup:
	kfree(dev->eps[n]);
	dev->eps[n] = NULL;

	return status;
}

static void bridge_configuration(struct usb_gadget *gadget, int index) {
	struct proxy_dev	*dev = get_gadget_data(gadget);
	int i, j, k;
	struct usb_host_config* config;
	struct usb_host_interface *alt = NULL;

	for (i = 0; i < USB_MAXENDPOINT; i++) {
		if (dev->eps[i]) {
			device_free_ep(dev, dev->eps[i]);
			gadget_free_ep(dev->eps[i]);
			kfree(dev->eps[i]);
			dev->eps[i] = NULL;
		}
	}

	DBG(10, "Bridging endpoints.\n");

	for (i = 0; i < dev->udev->descriptor.bNumConfigurations; i++) {
		config = &dev->udev->config[i];
		DBG(15, "Config %d (%d)\n",
		    i, config->desc.bConfigurationValue);

		if (config->desc.bConfigurationValue != index)
			continue;

		DBG(15, "Got the config!\n");

		for (j = 0; j < config->desc.bNumInterfaces; j++) {
			/* Set up endpoints for alternate interface setting 0 */
			alt = usb_find_alt_setting(config, j, 0);
			if (!alt)
				/* No alt setting 0? Pick the first setting. */
				alt = &config->intf_cache[j]->altsetting[0];
			
			DBG(15, "Interface %d: %d endpoints\n", j,
				alt->desc.bNumEndpoints);

			/* FIXME: usb_enable_interface almost does everything we want */
			//config->interface[j]->cur_altsetting = alt;
			//usb_enable_interface(dev->udev, config->interface[j], true);

			for (k = 0; k < alt->desc.bNumEndpoints; k++) {
				DBG(15, "EP %d, bridging...\n", k);
				bridge_endpoint(gadget, &alt->endpoint[k]);
				DBG(15, "EP %d, Done!\n", k);
			}
		}

		break;
	}
}

/*-------------------------------------------------------------------------*/
/* USB device part */

/* Callback, when a ctrl request has come back from the device,
 * either IN, or OUT with 0 length. */
static void device_setup_complete(struct urb *urb) {
	struct usb_gadget	*gadget = urb->context; 
	struct proxy_dev	*dev = get_gadget_data(gadget);
	struct usb_request	*req = dev->ep0.req;
	struct usb_ctrlrequest	*ctrl = dev->ep0.ctrlreq;
	u16			wValue = le16_to_cpu(ctrl->wValue);
	u16			wIndex = le16_to_cpu(ctrl->wIndex);
	int			status = -EOPNOTSUPP;
	int			index;

	DBG(20, "Device setup complete (%d; %d)!",
			urb->status, urb->actual_length);

	DBG(20, "ctrl req%02x(%02x).%02x v%04x\n",
	    ctrl->bRequestType, ctrl->bRequestType&USB_TYPE_MASK,
	    ctrl->bRequest, wValue);

	if ((ctrl->bRequestType == USB_DIR_IN) &&
	    (ctrl->bRequest == USB_REQ_GET_DESCRIPTOR)) {
		DBG(15, "GET_DESCRIPTOR (%04x)\n", wValue);
		switch (wValue >> 8) {
		case USB_DT_DEVICE:
			DBG(15, "DT_DEVICE\n");
			/* TODO: In theory, I should not update that field, but
			 * the MUSB driver only supports 64 bytes.
			 * In Full-speed, we could support 8, 16, 32, 64 bytes
			 * (8 is what low-speed needs).
			 * See USB 2.0 specs, 5.5.3. */

			((struct usb_device_descriptor*)req->buf)->bMaxPacketSize0
				= gadget->ep0->maxpacket;
			break;
		case USB_DT_CONFIG:
			index = wValue & 0xFF;
			DBG(15, "DT_CONFIG (%d)\n", index);

			if (index >= dev->udev->descriptor.bNumConfigurations) {
				ERROR("Config %d, only %d were expected!\n",
					index, dev->udev->descriptor.bNumConfigurations);
				break;
			}

			/* Note: on the first pass, the configuration descriptor
			 * is incomplete, and usb_parse_configuration display
			 * an error. This is harmless (the full descriptor is
			 * requested just after that). */
			status = usb_parse_configuration(&dev->udev->dev, index,
				   &dev->udev->config[index], req->buf,
				   urb->actual_length);
			if (status < 0) {
				ERROR("Parse configuration error\n");
			}
			rewrite_config(req->buf, urb->actual_length);
			break;
		}
	}

	/* TODO: Not sure what to do if urb->status < 0... */
	req->complete = gadget_setup_complete;
	req->length = urb->actual_length;
	req->zero = 0; //value < wLength;
	status = usb_ep_queue(gadget->ep0, req, GFP_ATOMIC);
	DBG(20, "device_setup_complete ep_queue --> %d\n", status);
	if (status < 0) {
		ERROR("device_setup_complete ep_queue --> %d\n", status);
		req->status = 0;
		gadget_setup_complete(gadget->ep0, req);
	}

	/* If SET_CONFIGURATION has been sent to the host,
	 * initialize the other endpoints. */
	if ((ctrl->bRequestType&USB_TYPE_MASK) == USB_TYPE_STANDARD) {
		switch (ctrl->bRequest) {
		case USB_REQ_SET_CONFIGURATION:
			DBG(15, "SET_CONFIGURATION\n");

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

			bridge_configuration(gadget, wValue & 0xFF);
			//}
			break;
		case USB_REQ_SET_INTERFACE:
			DBG(1, "SET_INTERFACE (if=%d, alt=%d)\n", wIndex, wValue);

			/* FIXME: Handle that */

			break;
		case USB_REQ_SYNCH_FRAME:
			DBG(1, "SYNCH_FRAME (ep=%d)\n", wIndex);

			break;
		}
	}
}

static void device_setup_out_complete(struct urb *urb) {
	struct proxy_dev *dev = &usb_proxy_gadget;

	DBG(20, "Device setup OUT complete (%d; %d)!\n",
	    urb->status, urb->actual_length);

	dev->ep0.flying -= 1;
	/* Handle the next ctrl request, if any was buffered. */
	if (dev->ep0.flying > 0)
		gadget_handle_ctrlrequest(dev->gadget, dev->ep0.ctrlreq_pending);
}

/* Device IN endpoint, got some data from the device,
 * forward data to the host. */
static void device_epin_irq(struct urb *urb)
{
	struct proxy_endpoint *ep = urb->context;
	int status;	

	switch (urb->status) {
	case 0:			/* success */
		break;
	case -ECONNRESET:	/* unlink */
	case -ENOENT:
	case -ESHUTDOWN:
		ERROR("EP-IN error: status %d, not resubmitting\n",
			urb->status);
		return;
	/* -EPIPE:  should clear the halt */
	default:		/* error */
		ERROR("EP-IN error: status %d, resubmitting\n", urb->status);
		goto resubmit;
	}

	DBG(30, "Got an EP-IN urb back (%d)!\n", urb->actual_length);

	memcpy(ep->req->buf, urb->transfer_buffer, urb->actual_length);

	ep->req->length = urb->actual_length;
	ep->req->zero = 0;
	
	status = usb_ep_queue(ep->gadget_ep, ep->req, GFP_ATOMIC);
	DBG(30, "device_epin_irq ep_queue --> %d\n", status);
	if (status < 0) {
		ERROR("device_epin_irq ep_queue --> %d\n", status);
		ep->req->status = 0;
		gadget_epin_complete(ep->gadget_ep, ep->req);
	}

	return;

resubmit:
	; /* Resubmission is done in proxy_ep1_complete normally */
	status = usb_submit_urb (urb, GFP_ATOMIC);
	if (status)
		ERROR("can't resubmit urb, status %d\n", status);
}

/* Device OUT endpoint, data was sent properly to the device,
 * resubmit request, to receive next packet from the host. */
static void device_epout_irq(struct urb *urb) {
	struct proxy_endpoint *ep = urb->context;

	int status;

	DBG(25, "EP-OUT IRQ callback\n");

	status = usb_ep_queue(ep->gadget_ep, ep->req, GFP_ATOMIC);
	DBG(25, "epout_queue --> %d\n", status);
	if (status < 0) {
		ERROR("device_epout_irq ep_queue --> %d\n", status);
		ep->req->status = 0;
	}
}

/* Connect an endpoint, on the device side */
static int device_setup_ep(struct proxy_dev *dev, struct proxy_endpoint* ep) {
	struct usb_device *udev = dev->udev;
	int pipe;
	u16 wMaxPacketSize = le16_to_cpu(ep->desc->wMaxPacketSize);
	void* data;
	usb_complete_t complete_fn;
	int status = 0;

	INFO("Connecting EP...\n");

	usb_enable_endpoint(dev->udev, ep->device_ep, true);

	ep->urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!ep->urb) {
		status = -ENOMEM;
		goto fail_alloc_urb;
	}

	/* FIXME: is it correct to allocate only wMaxPacketSize? */
	data = usb_buffer_alloc(udev, wMaxPacketSize,
				GFP_ATOMIC, &ep->urb->transfer_dma);
	if (!data) {
		status = -ENOMEM;
		goto fail_buffer_alloc;
	}

	ep->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	if (usb_endpoint_dir_in(ep->desc))
		complete_fn = device_epin_irq;
	else
		complete_fn = device_epout_irq;

	if (usb_endpoint_xfer_int(ep->desc)) {
		if (usb_endpoint_dir_in(ep->desc))
			pipe = usb_rcvintpipe(udev, ep->desc->bEndpointAddress);
		else
			pipe = usb_sndintpipe(udev, ep->desc->bEndpointAddress);

		/* Setup an urb on the device side */
		usb_fill_int_urb(ep->urb, udev, pipe, data, wMaxPacketSize,
				 complete_fn, ep, ep->desc->bInterval);
	} else if (usb_endpoint_xfer_bulk(ep->desc)) {
		if (usb_endpoint_dir_in(ep->desc))
			pipe = usb_rcvbulkpipe(udev, ep->desc->bEndpointAddress);
		else
			pipe = usb_sndbulkpipe(udev, ep->desc->bEndpointAddress);

		/* Setup an urb on the device side */
		usb_fill_bulk_urb(ep->urb, udev, pipe, data, wMaxPacketSize,
				  complete_fn, ep);
	} else {
		ERROR("Unknown transfer type for EP\n");
		return -EINVAL;
	}

	return 0;

fail_buffer_alloc:
	usb_free_urb(ep->urb);
	ep->urb = NULL;
fail_alloc_urb:
	usb_disable_endpoint(dev->udev, ep->desc->bEndpointAddress, true);

	return status;
}

static void device_free_ep(struct proxy_dev *dev, struct proxy_endpoint* ep) {
	if (ep->device_ep) {
		/* Disabling the endpoint by address seems a bit dirty... */
		usb_disable_endpoint(dev->udev, ep->desc->bEndpointAddress, true);
		ep->device_ep = NULL;
	}

	if (ep->urb) {
		usb_kill_urb(ep->urb);
		usb_buffer_free(dev->udev,
				le16_to_cpu(ep->desc->wMaxPacketSize),
				ep->urb->transfer_buffer,
				ep->urb->transfer_dma);
		usb_free_urb(ep->urb);
		ep->urb = NULL;
	}
}

static int device_probe(struct usb_device *udev) {
	int status;
	struct proxy_dev *dev = &usb_proxy_gadget;
	int ncfg = udev->descriptor.bNumConfigurations;

	INFO("device_probe (%04x:%04x)\n",
		le16_to_cpu(udev->descriptor.idVendor),
		le16_to_cpu(udev->descriptor.idProduct));

	if (dev->udev) {
		INFO("Already attached to another device!\n");
		return -1;
	}

	if (ncfg > USB_MAXCONFIG) {
		WARNING("too many configurations: %d, "
		    "using maximum allowed: %d\n", ncfg, USB_MAXCONFIG);
		udev->descriptor.bNumConfigurations = ncfg = USB_MAXCONFIG;
	}

	if (ncfg < 1) {
		ERROR("no configurations\n");
		return -EINVAL;
	}

	dev->ep0.ctrlreq = kmalloc(sizeof(*dev->ep0.ctrlreq), GFP_KERNEL);
	if (!dev->ep0.ctrlreq) {
		status = -ENOMEM;
		goto fail_ctrlreq;
	}

	dev->ep0.ctrlreq_pending = kmalloc(sizeof(*dev->ep0.ctrlreq_pending), GFP_KERNEL);
	if (!dev->ep0.ctrlreq_pending) {
		status = -ENOMEM;
		goto fail_ctrlreq_pending;
	}

	dev->ep0.urb = usb_alloc_urb(0, GFP_ATOMIC);
	if (!dev->ep0.urb) {
		status = -ENOMEM;
		goto fail_urb;
	}

	if (udev->config)
		kfree(udev->config);

	udev->config = kzalloc(ncfg * sizeof(struct usb_host_config), GFP_KERNEL);
	if (!udev->config) {
		status = -ENOMEM;
		goto fail_config;
	}

	dev->udev = udev;

	INFO("Attaching!\n");
	if (udev->speed == USB_SPEED_HIGH) {
		proxy_gadget_driver.speed = USB_SPEED_HIGH;
		INFO("Using high speed.\n");
	}
	else { /* LOW or FULL: use full */
		proxy_gadget_driver.speed = USB_SPEED_FULL;
		INFO("Using full speed.\n");
	}

	status = usb_gadget_register_driver(&proxy_gadget_driver);
	if (status) {
		ERROR("usb_gadget_register_driver failed %d\n", status);
		goto fail_attach;
	}

	return 0;

fail_attach:
	kfree(udev->config);
	udev->config = NULL;
fail_config:
	usb_free_urb(dev->ep0.urb);
fail_urb:
	kfree(dev->ep0.ctrlreq_pending);
fail_ctrlreq_pending:
	kfree(dev->ep0.ctrlreq);
fail_ctrlreq:
	return status;
}

static void device_disconnect(struct usb_device *udev) {
	struct proxy_dev	*dev;
	int status, i;
	dev = &usb_proxy_gadget;

	INFO("device_disconnect\n");

	kfree(udev->config);
	udev->config = NULL;
	usb_kill_urb(dev->ep0.urb);
	usb_free_urb(dev->ep0.urb);
	kfree(dev->ep0.ctrlreq_pending);
	kfree(dev->ep0.ctrlreq);

	for (i = 0; i < USB_MAXENDPOINT; i++) {
		if (dev->eps[i])
			device_free_ep(dev, dev->eps[i]);
	}

	status = usb_gadget_unregister_driver(&proxy_gadget_driver);
	if (status)
		ERROR("usb_gadget_unregister_driver %d\n", status);

	/* Go back to a clean state. */
	memset(&proxy_gadget_driver, 0, sizeof(proxy_gadget_driver));
}

static int device_suspend(struct usb_device *udev, pm_message_t message) {
	INFO("device_suspend\n");

	return 0;
}

static int device_resume(struct usb_device *udev, pm_message_t message) {
	INFO("device_resume\n");

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

static int __init
init(void)
{
	int status;

	INFO("%s, version: " DRIVER_VERSION " debug=%d\n", driver_desc, debug);

	/* register this driver with the USB subsystem */
	status = usb_register_device_driver(&proxy_device_driver, THIS_MODULE);
	if (status) {
		ERROR("usb_register failed. Error number %d", status);
		return status;
	}

	return status;
}
module_init(init);

static void __exit
cleanup(void)
{
	DBG(5, "%s\n", __func__);

	usb_deregister_device_driver(&proxy_device_driver);
}
module_exit(cleanup);
