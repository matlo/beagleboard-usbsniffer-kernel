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

/* BUG: gadget disc/reconn. */
/* BUG: alloc'ed bandwidth is not freed upon driver reload (maybe?). */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/smp_lock.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/moduleparam.h>
#include <linux/workqueue.h>

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
	xprintk(KERN_ERR, "ERROR(%s): " fmt, __func__, ## args)
#define WARNING(fmt, args...) \
	xprintk(KERN_WARNING, fmt, ## args)
#define INFO(fmt, args...)			\
	xprintk(KERN_INFO, fmt, ## args)

/*-------------------------------------------------------------------------*/

#define USB_MAXCONFIG                  8      /* Arbitrary limit */
#define USB_MAXENDPOINT               32      /* 15 IN, 15 OUT */
#define USB_DESC_BUFSIZE             512

/*-------------------------------------------------------------------------*/

enum proxy_request_state {
        PREQ_STATE_READY = 0,	/* Not in use */
        PREQ_STATE_SUBMITTED,	/* urb/req was submitted to the driver
				   (and we don't necessarily expect
				   a quick callback) */
        PREQ_STATE_BUSY       	/* request in use. */
};

struct proxy_ctrlrequest {
	struct usb_ctrlrequest  ctrl;
	struct list_head	list;
};

/* request/urb pair */
struct proxy_request {
	struct usb_request	*req;
	struct urb		*urb;

	struct proxy_endpoint	*ep;

	enum proxy_request_state state;

	struct proxy_ctrlrequest  *ctrl;

	/* For queueing SET_CONFIGURATION/SET_INTERFACE requests,
	 * FIXME: this bloats all requests... */
	struct work_struct	work;

	struct list_head	list;
};

struct proxy_endpoint {
	struct usb_endpoint_descriptor* desc;

	struct usb_ep		*gadget_ep;

	struct usb_host_endpoint *device_ep;

	struct list_head	req_list;
};

struct proxy_dev {
	spinlock_t		lock;	  /* lock this structure (FIXME: useless for now) */
	struct usb_gadget	*gadget;

	struct usb_device	*udev;	  /* the usb device for this device */

	/* EP0 specific */
	struct proxy_endpoint   ep0;
	struct list_head        ctrl_list;

	/* Other EPs stuff */
	struct proxy_endpoint	*eps[USB_MAXENDPOINT];

	/* We cannot use udev->actconfig (usb_disable_device gets confused),
	 * so replicate. */
	struct usb_host_config  *actconfig;
};

/*-------------------------------------------------------------------------*/

static struct proxy_dev usb_proxy_gadget;

static void proxy_disable_ep(struct proxy_endpoint* ep, bool reset_hardware);

static int gadget_handle_setup(void);
static void device_setup_complete(struct urb *urb);
static void device_setup_out_complete(struct urb *urb);

static void device_epin_irq(struct urb *urb);
static void device_epout_irq(struct urb *urb);

static void gadget_epin_complete(struct usb_ep *ep, struct usb_request *req);
static void gadget_epout_complete(struct usb_ep *ep, struct usb_request *req);

static void handle_set_conf_intf(struct work_struct *work);

/*-------------------------------------------------------------------------*/
/* Gadget side of the driver */

#define NBUFFER 8

/* Allocate and free proxy_request (blob of urb+request) */
static struct proxy_request*
proxy_req_alloc(struct proxy_endpoint *ep, gfp_t gfp_flags, bool ep0)
{
	struct proxy_dev* dev = &usb_proxy_gadget;
	struct proxy_request *preq;
	u16 len;
	usb_complete_t complete_fn;
	int pipe;
	int eptype, epdir;
	int packetsize;
	int nbuffer, i;

	if (!ep) {
		ERROR("ep is NULL.\n");
		return NULL;
	}

	if (!ep0 && !ep->desc) {
		ERROR("ep->desc is NULL, and ep != 0.\n");
		return NULL;
	}

	if (!ep->gadget_ep) {
		ERROR("ep->gadget_ep is NULL.\n");
		return NULL;
	}

	/* ep->desc is NULL if ep0, it is not used after that. */
	if (ep0) {
		packetsize = len = USB_DESC_BUFSIZE;
		eptype = USB_ENDPOINT_XFER_CONTROL;
		epdir = 0; /* does not matter */
		nbuffer = 0;
	} else {
		eptype = usb_endpoint_type(ep->desc);
		epdir = ep->desc->bEndpointAddress & USB_ENDPOINT_DIR_MASK;

		/* FIXME: For ISOC endpoints, only one buffer (for now...) */
		/* FIXME: For INT endpoints, >1 causes some lag... */
		nbuffer = (eptype == USB_ENDPOINT_XFER_BULK) ? NBUFFER : 1;
		/* TODO: Handle high-speed 3072 packetsize */
		packetsize = le16_to_cpu(ep->desc->wMaxPacketSize);
		len = nbuffer*packetsize;
	}

	preq = kzalloc(sizeof(*preq), gfp_flags);
	if (!preq)
		goto fail;

	preq->ep = ep;

	preq->req = usb_ep_alloc_request(ep->gadget_ep, gfp_flags);
	if (!preq->req)
		goto fail_req;

	/* FIXME: is it correct to allocate only wMaxPacketSize? */
	preq->req->length = len;
	preq->req->buf = kmalloc(len, gfp_flags);
	if (!preq->req->buf)
		goto fail_buf;
	
	if (!ep0)
		preq->req->complete = (epdir == USB_DIR_IN) ?
			gadget_epin_complete : gadget_epout_complete;
	preq->req->context = preq;

	preq->req->length = len;
	preq->req->zero = 0;

	INIT_WORK(&preq->work, handle_set_conf_intf);

	DBG(50, "proxy_req_alloc: usb_alloc_urb\n");

	/* Alloc URB */
	preq->urb = usb_alloc_urb((eptype == USB_ENDPOINT_XFER_ISOC) ? nbuffer : 0, gfp_flags);
	if (!preq->urb)
		goto fail_alloc_urb;
	
	/* TODO: this is for DMA, but then we need to use
	 * usb_buffer_alloc. */
	//ep->preq.urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
	
	complete_fn = (epdir == USB_DIR_IN) ?
		device_epin_irq : device_epout_irq;

	switch (eptype) {
	case USB_ENDPOINT_XFER_CONTROL:
		/* TODO: Fill control URB here, maybe... */
		break; 
	case USB_ENDPOINT_XFER_INT:
		pipe = (epdir == USB_DIR_IN) ?
			usb_rcvintpipe(dev->udev, ep->desc->bEndpointAddress) :
			usb_sndintpipe(dev->udev, ep->desc->bEndpointAddress);

		/* Setup an urb on the device side */
		usb_fill_int_urb(preq->urb, dev->udev, pipe, preq->req->buf, len,
				 complete_fn, preq, ep->desc->bInterval);
		break;
	case USB_ENDPOINT_XFER_BULK:
		pipe = (epdir == USB_DIR_IN) ?
			usb_rcvbulkpipe(dev->udev, ep->desc->bEndpointAddress) :
			usb_sndbulkpipe(dev->udev, ep->desc->bEndpointAddress);

		/* Setup an urb on the device side */
		usb_fill_bulk_urb(preq->urb, dev->udev, pipe, preq->req->buf, len,
				  complete_fn, preq);
		break;
	case USB_ENDPOINT_XFER_ISOC:
		pipe = (epdir == USB_DIR_IN) ?
			usb_rcvisocpipe(dev->udev, ep->desc->bEndpointAddress) :
			usb_sndisocpipe(dev->udev, ep->desc->bEndpointAddress);

			/*preq->urb->transfer_buffer = usb_buffer_alloc(dev->udev,
							len,
							GFP_KERNEL,
							&preq->urb->transfer_dma);*/
		preq->urb->transfer_buffer = preq->req->buf;

		/* From usb.h:usb_fill_int_urb */
		preq->urb->dev = dev->udev;
		preq->urb->pipe = pipe;
		preq->urb->transfer_buffer_length = len;
		preq->urb->complete = complete_fn;
		preq->urb->context = preq;
		if (dev->udev->speed == USB_SPEED_HIGH)
			preq->urb->interval = 1 << (ep->desc->bInterval - 1);
		else
			preq->urb->interval = ep->desc->bInterval;
		preq->urb->start_frame = 0; /* FIXME: ? */
		preq->urb->transfer_flags = URB_ISO_ASAP;
		//preq->urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

		preq->urb->number_of_packets = nbuffer;
		for (i = 0; i < nbuffer; i++) {
			preq->urb->iso_frame_desc[i].offset = i*packetsize;
			preq->urb->iso_frame_desc[i].length = packetsize;
		}

		break;
	default:
		ERROR("Unknown transfer type for EP\n");
		goto fail_fill;
	}

	return preq;

fail_fill:
	usb_free_urb(preq->urb);
fail_alloc_urb:
	kfree(preq->req->buf);
fail_buf:
	usb_ep_free_request(ep->gadget_ep, preq->req);
fail_req:
	kfree(preq);
fail:
	return NULL;
}

static void
proxy_req_free(struct proxy_request *preq)
{
	DBG(10, "proxy_req_free (%p)\n", preq);

	if (!preq->ep->desc) /* ep0 */
		cancel_work_sync(&preq->work);

	usb_ep_dequeue(preq->ep->gadget_ep, preq->req);
	usb_kill_urb(preq->urb);

	kfree(preq->req->buf);
	usb_ep_free_request(preq->ep->gadget_ep, preq->req);
	usb_free_urb(preq->urb);

	kfree(preq);
}

/* EP operations callbacks */

/* EP0 IN or 0-length OUT callback. */
static void gadget_setup_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct proxy_request *preq = req->context;

	DBG(15, "setup complete --> %d, %d/%d\n",
	    req->status, req->actual, req->length);

	preq->state = PREQ_STATE_READY;
	kfree(preq->ctrl);
	gadget_handle_setup();
}

/* EP0 OUT with data callback, send the data the we just read to the device */
static void gadget_setup_out_complete(struct usb_ep *ep, struct usb_request *req)
{
	int status, pipe;
	struct proxy_dev *dev = ep->driver_data;
	struct proxy_request *preq = req->context;

	DBG(20,
	    "setup out complete --> %d, %d/%d\n",
	    req->status, req->actual, req->length);
	
	pipe = usb_sndctrlpipe(dev->udev, 0);
		
	/* FIXME: check buf and length (there could be some overruns) */
	usb_fill_control_urb(preq->urb, dev->udev, pipe, (void*)preq->ctrl,
			     req->buf, req->actual,
			     device_setup_out_complete, preq);
	/* TODO: Understand how the DMA works... */
	//urb->setup_dma = usbhid->cr_dma;

	status = usb_submit_urb(preq->urb, GFP_ATOMIC);
	DBG(20, "EP0 OUT URB submitted (%d)...\n", status);
	if (status) {
		ERROR("can't submit EP0 OUT URB, status %d\n", status);
		/* TODO: Can we do something here? */
		preq->state = PREQ_STATE_READY;
		kfree(preq->ctrl);
		gadget_handle_setup();
		return;
	}
}


/* Gadget IN endpoint. Data was sent to the host,
 * resubmit the urb to the device. */
static void gadget_epin_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct proxy_request *preq = req->context;
	int status;

	if (preq->state != PREQ_STATE_BUSY) {
		WARNING("gadget_epin_complete: (%p): invalid state (%d)",
			preq, preq->state);
	}
/*
	if (!strcmp(preq->ep->gadget_ep->name, "ep2in")) {
		WARNING("not resubmitting ep2in, hehe so mean...\n");
		return;
	}
*/

	if (!strcmp(preq->ep->gadget_ep->name, "ep1in")) {
		if (req->length > 0) {
			DBG(2, "(%p//%p - %s) device_epin_complete (%d)\n",
				preq, preq->req, preq->ep->gadget_ep->name,
				req->status);
			//WARN_ON(1);
		}
	}

	preq->state = PREQ_STATE_SUBMITTED;

	switch (req->status) {
	case 0:			/* success */
		break;
	case -ECONNRESET:	/* unlink */
	case -ENOENT:
	case -ESHUTDOWN:
		ERROR("EP-IN error: status %d, not resubmitting\n",
			req->status);
		preq->state = PREQ_STATE_READY;
		return;
	/* -EPIPE:  should clear the halt (FIXME: Was does that mean?) */
	default:		/* error */
		ERROR("EP-IN error: status %d, resubmitting\n", req->status);
	}

	DBG(30, "EP-IN complete --> %d, %d/%d\n",
	    req->status, req->actual, req->length);

	preq->urb->dev = usb_proxy_gadget.udev;
	status = usb_submit_urb(preq->urb, GFP_ATOMIC);
	if (status) {
		ERROR("can't resubmit URB, status %d\n", status);
		/* TODO: Can we do something here? */
		preq->state = PREQ_STATE_READY;
	}
}

/* Gadget OUT endpoint. Data came from the host, forward to the device */
static void gadget_epout_complete(struct usb_ep *ep, struct usb_request *req)
{
	struct proxy_request *preq = req->context;
	int status;

	preq->state = PREQ_STATE_BUSY;

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
		preq->state = PREQ_STATE_READY;
		return;
	/* -EPIPE:  should clear the halt (FIXME: Was does that mean?) */
	default:		/* error */
		ERROR("EP-OUT error: status %d, resubmitting\n", req->status);
		goto resubmit;
	}

	//memcpy(preq->urb->transfer_buffer, req->buf, req->actual);
	//preq->urb->transfer_buffer = req->buf;

	preq->urb->transfer_buffer_length = req->actual;

	status = usb_submit_urb(preq->urb, GFP_ATOMIC);
	DBG(25, "EP-OUT submit urb --> %d\n", status);
	if (status < 0) {
		ERROR("gadget_epout_complete submit urb --> %d\n", status);
		preq->state = PREQ_STATE_READY;
	}

	return;

resubmit:
	preq->state = PREQ_STATE_SUBMITTED;

	status = usb_ep_queue(ep, req, GFP_ATOMIC);
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
gadget_setup(struct usb_gadget *gadget,
			 const struct usb_ctrlrequest *ctrl)
{
	struct proxy_dev	*dev = get_gadget_data(gadget);
	struct proxy_ctrlrequest *ctrlreq;

	ctrlreq = kmalloc(sizeof(*ctrlreq), GFP_ATOMIC);

	/* ctrl is allocated on the stack in MUSB,
	   so copy it beforehand. */
	memcpy(&ctrlreq->ctrl, ctrl, sizeof(ctrlreq->ctrl));

	list_add_tail(&ctrlreq->list, &dev->ctrl_list);

	gadget_handle_setup();

	return 0;
}

static int gadget_handle_setup()
{
	struct usb_ctrlrequest  *ctrl;
	struct proxy_dev	*dev = &usb_proxy_gadget;
	struct proxy_ctrlrequest *pctrl;
	u16			wIndex;
	u16			wValue;
	u16			wLength;
	struct proxy_request	*preq = NULL, *ppreq;
	int status;
	unsigned int pipe;

	if (list_empty(&dev->ctrl_list)) {
		DBG(20, "No request to handle.\n");
		return 0;
	}

	pctrl = list_first_entry(&dev->ctrl_list, struct proxy_ctrlrequest, list);
	ctrl = &pctrl->ctrl;

	/* Find a free preq */
	list_for_each_entry (ppreq, &dev->ep0.req_list, list) {
		if (ppreq->state == PREQ_STATE_READY) {
			preq = ppreq;
			break;
		}
	}

	if (preq == NULL) {
		DBG(20, "No preq available to handle request.\n");
		return 0;
	}

	/* Pop the ctrl request from the list. */
	list_del(&pctrl->list);

	preq->state = PREQ_STATE_BUSY;
	preq->ctrl = pctrl;

	wIndex = le16_to_cpu(ctrl->wIndex);
	wValue = le16_to_cpu(ctrl->wValue);
	wLength = le16_to_cpu(ctrl->wLength);

	DBG(15, "handling ctrl req%02x(%02x).%02x v%04x i%04x l%d\n",
	    ctrl->bRequestType, ctrl->bRequestType&USB_TYPE_MASK,
	    ctrl->bRequest, wValue, wIndex, wLength);

	if (ctrl->bRequestType & USB_DIR_IN) {
		DBG(20, "DIR_IN\n");
		pipe = usb_rcvctrlpipe(dev->udev, 0);
	} else if (wLength == 0) {
		DBG(20, "DIR_OUT\n");
		pipe = usb_sndctrlpipe(dev->udev, 0);
	} else {
		DBG(20, "DIR_OUT, with data (%d)\n", wLength);
		
		preq->req->complete = gadget_setup_out_complete;
		preq->req->length = wLength;
		preq->req->zero = 0;
		preq->req->context = preq;
		status = usb_ep_queue(dev->gadget->ep0, preq->req, GFP_ATOMIC);
		DBG(20, "proxy_setup ep_queue --> %d\n", status);
		if (status < 0) {
			ERROR("DIR_OUT w/ data, proxy_setup ep_queue --> %d\n", status);
			/* Bail out... */
			preq->state = PREQ_STATE_READY;
			return status;
		}

		return 0;
	}
	
	/* FIXME: check buf and wLength (there could be some overruns) */
	usb_fill_control_urb(preq->urb, dev->udev, pipe,
			(void*)preq->ctrl, preq->req->buf, wLength,
			device_setup_complete, preq);
	/* TODO: Understand how the DMA works... */
	//urb->setup_dma = usbhid->cr_dma;
		
	status = usb_submit_urb(preq->urb, GFP_ATOMIC);
	DBG(20, "URB submitted (%d)...\n", status);
	if (status < 0) {
		ERROR("proxy_handle_ctrlrequest submit urb --> %d\n", status);
		preq->state = PREQ_STATE_READY;
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

	proxy_disable_ep(&dev->ep0, true);

	/* TODO: Ideally, we should dequeue reqs, or does it come for
	 * free when we flush the FIFOs? */

	for (i = 0; i < USB_MAXENDPOINT; i++) {
		if (dev->eps[i]) {
			proxy_disable_ep(dev->eps[i], true);
			kfree(dev->eps[i]);
			dev->eps[i] = NULL;
		}
	}

	set_gadget_data(gadget, NULL);
}

static int __init
gadget_bind(struct usb_gadget *gadget)
{
	struct proxy_dev	*dev= &usb_proxy_gadget;
	int			status = -ENOMEM;
	int i;
	struct proxy_request *preq;

	DBG(5, "%s\n", __func__);

	/* TODO: Warning if !MUSB */

	/* Reset host EPs. */
	usb_ep_autoconfig_reset(gadget);

	spin_lock_init(&dev->lock);

	dev->ep0.gadget_ep = gadget->ep0;

	/* preallocate control message data and buffer */
	INIT_LIST_HEAD(&dev->ep0.req_list);

	/* Only _one_ preq is needed. */
	for (i = 0; i < 1; i++) {
		preq = proxy_req_alloc(&dev->ep0, GFP_KERNEL, true);
		if (!preq) {
			status = -ENOMEM;
			goto fail_req_alloc;
		}

		list_add(&preq->list, &dev->ep0.req_list);
	}

	INIT_LIST_HEAD(&dev->ctrl_list);

	/* finish hookup to lower layer ... */
	dev->gadget = gadget;
	set_gadget_data(gadget, dev);
	gadget->ep0->driver_data = dev;

	INFO("using %s\n", gadget->name);

	return 0;

fail_req_alloc:
	while (!list_empty(&dev->ep0.req_list)) {
		preq = list_first_entry(&dev->ep0.req_list,
					struct proxy_request, list);
		list_del(&preq->list);
		proxy_req_free(preq);
	}
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

static int proxy_enable_ep(struct proxy_endpoint* ep)
{
	struct proxy_dev *dev = &usb_proxy_gadget;
	int status;
	
	status = usb_ep_enable(ep->gadget_ep, ep->desc);
	if (status) {
		ERROR("can't enable %s, result %d\n",
			ep->gadget_ep->name, status);
		return status;
	}

	/* Device side */
	usb_enable_endpoint(dev->udev, ep->device_ep, true);
	
	return 0;
}

static void proxy_disable_ep(struct proxy_endpoint* ep, bool reset_hardware)
{
	struct proxy_dev *dev = &usb_proxy_gadget;
	struct proxy_request *preq;

	while (!list_empty(&ep->req_list)) {
		preq = list_first_entry(&ep->req_list,
					struct proxy_request, list);
		list_del(&preq->list);
		
		proxy_req_free(preq);
	}

	if (ep->gadget_ep) {
		usb_ep_fifo_flush(ep->gadget_ep);
		if (ep->desc) /* Only EP0 doesn't have a desc */
			usb_ep_disable(ep->gadget_ep);
	}

	if (ep->desc) {
		/* Disabling the endpoint by address seems a bit dirty... */
		usb_disable_endpoint(dev->udev, ep->desc->bEndpointAddress, reset_hardware);
	}
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
	struct proxy_request *preq;
	struct proxy_dev *dev = get_gadget_data(gadget);
	int n;
	int status;
	int i;
	int wMaxPacketSize = le16_to_cpu(desc->wMaxPacketSize);

	DBG(1, "Endpoint: %02x (%s %s), maxPacket=%d\n",
	    desc->bEndpointAddress,
	    usb_endpoint_dir_in(desc) ? "IN": "OUT",
	    usb_endpoint_xfer_bulk(desc) ? "bulk" :
	    usb_endpoint_xfer_control(desc) ? "control" :
	    usb_endpoint_xfer_int(desc) ? "int" :
	    usb_endpoint_xfer_isoc(desc) ? "isoc" :
	    "???",
	    wMaxPacketSize);

	if (wMaxPacketSize == 0) {
		DBG(1, "Disabled EP, not allocating.\n");
		return -EINVAL;
	}
	
	for (n = 0; n < USB_MAXENDPOINT; n++) {
		if (!dev->eps[n])
			break;
	}

	if (n == USB_MAXENDPOINT) {
		ERROR("Too many endpoints!\n");
		return -EINVAL;
	}

	DBG(1, "Using dev->eps[%d].\n", n);

	dev->eps[n] = kzalloc(sizeof(struct proxy_endpoint), GFP_KERNEL);
	if (!dev->eps[n])
		return -ENOMEM;

	dev->eps[n]->device_ep = ep;
	dev->eps[n]->desc = &ep->desc;

	/* Gadget side */
	dev->eps[n]->gadget_ep = find_gadget_endpoint(dev->gadget, dev->eps[n]->desc);
	if (!dev->eps[n]->gadget_ep) {
		ERROR("Can't find endpoint in gadget driver.\n");
		status = -ENOENT;
		goto error_req_alloc;
	}
	dev->eps[n]->gadget_ep->driver_data = dev;	/* claim */

	INIT_LIST_HEAD(&dev->eps[n]->req_list);

	for (i = 0; i < 32; i++) {
		preq = proxy_req_alloc(dev->eps[n], GFP_KERNEL, false);
		if (!preq) {
			status = -ENOMEM;
			goto error_req_alloc;
		}

		list_add(&preq->list, &dev->eps[n]->req_list);
	}

	/* Init EP on both side... */
	status = proxy_enable_ep(dev->eps[n]);
	if (status < 0) {
		ERROR("cannot enable EP (%d).\n", status);
		goto error_enable_ep;
	}

	/* For IN endpoint, submit the urb, so that we can receive transfers. */
	if (usb_endpoint_dir_in(dev->eps[n]->desc)) {
		list_for_each_entry(preq, &dev->eps[n]->req_list, list) {
			preq->state = PREQ_STATE_SUBMITTED;

			DBG(-1, "submitting URB...\n");

			status = usb_submit_urb(preq->urb, GFP_KERNEL);

			if (status < 0) {
				ERROR("can't submit EP urb, status %d\n", status);
				/* bail out */
				goto error_submit;
			}
		}		
	} else { /* For OUT endpoint, submit the request. */
		list_for_each_entry(preq, &dev->eps[n]->req_list, list) {
			preq->state = PREQ_STATE_SUBMITTED;

			status = usb_ep_queue(dev->eps[n]->gadget_ep, preq->req, GFP_KERNEL);

			if (status < 0) {
				ERROR("ep_queue --> status %d\n", status);
				/* bail out */
				goto error_submit;
			}
		}
	}

	return 0;

error_submit:
	proxy_disable_ep(dev->eps[n], true);

error_enable_ep:
	while (!list_empty(&dev->eps[n]->req_list)) {
		preq = list_first_entry(&dev->eps[n]->req_list,
					struct proxy_request, list);
		list_del(&preq->list);
		proxy_req_free(preq);
	}

error_req_alloc:
	kfree(dev->eps[n]);
	dev->eps[n] = NULL;

	return status;
}

/* Copied from core/usb.c, but mine does not use dev->actconfig */
static struct usb_interface *proxy_usb_ifnum_to_if(unsigned ifnum)
{
	//struct usb_host_config *config = dev->actconfig;
	struct usb_host_config *config = usb_proxy_gadget.actconfig;
	int i;

	if (!config)
		return NULL;
	for (i = 0; i < config->desc.bNumInterfaces; i++)
		if (config->interface[i]->altsetting[0]
				.desc.bInterfaceNumber == ifnum)
			return config->interface[i];

	return NULL;
}

/* Heavily inspired from message.c:usb_set_interface */
/* Cannot be called from interrupt context (because of endpoints disabling) */
static int
bridge_interface(struct usb_gadget *gadget, int interface, int alternate)
{
	struct proxy_dev *dev = get_gadget_data(gadget);
	struct usb_interface *iface;
	struct usb_host_interface *alt;
	struct usb_host_interface *cur_alt;
	int i, k;
	int status;

	iface = proxy_usb_ifnum_to_if(interface);
	if (!iface) {
		DBG(5, "selecting invalid interface %d\n",
			interface);
		return -EINVAL;
	}

	alt = usb_altnum_to_altsetting(iface, alternate);
	if (!alt) {
		WARNING("selecting invalid altsetting %d",
			 alternate);
		return -EINVAL;
	}

	cur_alt = iface->cur_altsetting;
	/* FIXME: Find a disable endpoints */
	for (k = 0; k < cur_alt->desc.bNumEndpoints; k++) {
		DBG(15, "EP %d, disabling...\n", k);
		status = -1;
		for (i = 0; i < USB_MAXENDPOINT; i++) {
			if (dev->eps[i] && dev->eps[i]->desc == &cur_alt->endpoint[k].desc) {
				DBG(15, "Found EP %d (i=%d)...\n", k, i);
				proxy_disable_ep(dev->eps[i], false);
				kfree(dev->eps[i]);
				dev->eps[i] = NULL;
				status = 0;
				break;
			}
		}
		if (status < 0)
			WARNING("Cannot disable EP %d, not found!\n", k);

		DBG(15, "EP %d, Done!\n", k);
	}

	iface->cur_altsetting = alt;

	for (k = 0; k < alt->desc.bNumEndpoints; k++) {
		DBG(15, "EP %d, bridging...\n", k);
		bridge_endpoint(gadget, &alt->endpoint[k]);
		DBG(15, "EP %d, Done!\n", k);
	}

	status = usb_hcd_alloc_bandwidth(dev->udev, NULL, cur_alt, alt);
	if (status < 0) {
		ERROR("Cannot allocate bandwidth...\n");
		/* TODO: What to do? */
	}

	return 0;
}

/* Bits and pieces heavily inspired from message.c:usb_set_configuration */
/* Cannot be called from interrupt context (because of endpoints disabling) */
static void bridge_configuration(struct usb_gadget *gadget, int index)
{
	struct proxy_dev	*dev = get_gadget_data(gadget);
	int i, j, k, status;
	struct usb_host_config* config;
	struct usb_host_interface *alt = NULL;
	int nintf;

	for (i = 0; i < USB_MAXENDPOINT; i++) {
		if (dev->eps[i]) {
			proxy_disable_ep(dev->eps[i], false);
			kfree(dev->eps[i]);
			dev->eps[i] = NULL;
		}
	}

	DBG(10, "Bridging endpoints.\n");
	dev->actconfig = NULL;

	for (i = 0; i < dev->udev->descriptor.bNumConfigurations; i++) {
		config = &dev->udev->config[i];
		DBG(15, "Config %d (%d)\n",
		    i, config->desc.bConfigurationValue);

		if (config->desc.bConfigurationValue != index)
			continue;

		DBG(15, "Got the config!\n");

		/* Setting this breaks usb_disable_device */
		dev->actconfig = config;

		/* Alloc the bandwidth, first config. */
		status = usb_hcd_alloc_bandwidth(dev->udev, config, NULL, NULL);
		if (status < 0) {
			ERROR("Cannot allocate bandwidth...\n");
			/* TODO: What to do? */
		}

		nintf = config->desc.bNumInterfaces;

		for (j = 0; j < nintf; j++) {
			config->interface[j] = kzalloc(
				sizeof(struct usb_interface),
					GFP_KERNEL);
			/* FIXME: Sanity */

			config->interface[j]->altsetting = config->intf_cache[j]->altsetting;
			config->interface[j]->num_altsetting = config->intf_cache[j]->num_altsetting;
		}

		for (j = 0; j < nintf; j++) {
			/* Set up endpoints for alternate interface setting 0 */
			alt = usb_find_alt_setting(config, j, 0);
			if (!alt)
				/* No alt setting 0? Pick the first setting. */
				alt = &config->intf_cache[j]->altsetting[0];
			
			DBG(15, "Interface %d: %d endpoints\n", j,
				alt->desc.bNumEndpoints);

			/* FIXME: usb_enable_interface almost does everything we want */
			config->interface[j]->cur_altsetting = alt;
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

/* Workqueue handler */
static void handle_set_conf_intf(struct work_struct *work) {
	struct proxy_request* preq = container_of(work, struct proxy_request, work);
	struct proxy_dev	*dev = &usb_proxy_gadget;
	struct proxy_ctrlrequest *pctrl = preq->ctrl;
	struct usb_ctrlrequest	*ctrl = &pctrl->ctrl;
	u16			wValue = le16_to_cpu(ctrl->wValue);
	u16			wIndex = le16_to_cpu(ctrl->wIndex);
	int status;

	/* If SET_CONFIGURATION has been sent to the host,
	 * initialize the other endpoints. */
	if ((ctrl->bRequestType&USB_TYPE_MASK) == USB_TYPE_STANDARD) {
		switch (ctrl->bRequest) {
		case USB_REQ_SET_CONFIGURATION:
			DBG(15, "SET_CONFIGURATION\n");

			/* FIXME: Test reconfiguration. */

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

			bridge_configuration(dev->gadget, wValue & 0xFF);
			break;
		case USB_REQ_SET_INTERFACE:
			DBG(1, "SET_INTERFACE (if=%d, alt=%d)\n", wIndex, wValue);

			bridge_interface(dev->gadget, wIndex, wValue);

			break;
		case USB_REQ_SYNCH_FRAME:
			DBG(1, "SYNCH_FRAME (ep=%d)\n", wIndex);

			break;
		}
	}

	status = usb_ep_queue(dev->gadget->ep0, preq->req, GFP_KERNEL);
	DBG(15, "handle_set_conf_intf ep_queue --> %d\n", status);
	if (status < 0) {
		ERROR("handle_set_conf_intf ep_queue --> %d\n", status);
		preq->req->status = 0;
		gadget_setup_complete(dev->gadget->ep0, preq->req);
	}
}

/* Callback, when a ctrl request has come back from the device,
 * either IN, or OUT with 0 length. */
static void device_setup_complete(struct urb *urb) {
	struct proxy_dev	*dev = &usb_proxy_gadget;
	struct proxy_request	*preq = urb->context;
	struct proxy_ctrlrequest *pctrl = preq->ctrl;
	struct usb_ctrlrequest	*ctrl = &pctrl->ctrl;
	u16			wValue = le16_to_cpu(ctrl->wValue);
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

			((struct usb_device_descriptor*)preq->req->buf)->bMaxPacketSize0
				= dev->gadget->ep0->maxpacket;
			break;
		case USB_DT_CONFIG:
			index = wValue & 0xFF;
			DBG(15, "DT_CONFIG (%d)\n", index);

			if (index >= dev->udev->descriptor.bNumConfigurations) {
				ERROR("Config %d, only %d were expected!\n",
					index, dev->udev->descriptor.bNumConfigurations);
				break;
			}

			/* Note: on the first pass, the configuration
			 * descriptor is incomplete (only the first 8-9 bytes
			 * are requested), and usb_parse_configuration
			 * displays an error. This is harmless (the full
			 * descriptor is requested just after that).
			 */
			status = usb_parse_configuration(&dev->udev->dev, index,
				   &dev->udev->config[index], preq->req->buf,
				   urb->actual_length);
			if (status < 0) {
				ERROR("Parse configuration error\n");
			}
			rewrite_config(preq->req->buf, preq->urb->actual_length);
			break;
		}
	}

	/* TODO: Not sure what to do if urb->status < 0... */
	preq->req->complete = gadget_setup_complete;
	preq->req->length = urb->actual_length;
	preq->req->zero = 0; //value < wLength;
	preq->req->context = preq;

	if ((ctrl->bRequestType&USB_TYPE_MASK) == USB_TYPE_STANDARD &&
		(ctrl->bRequest == USB_REQ_SET_CONFIGURATION ||
			ctrl->bRequest == USB_REQ_SET_INTERFACE)) {
		/* In these cases, we need to sleep in the handler, so we
		 * schedule to a workqueue instead. */

		schedule_work(&preq->work);
		return; /* will be queued in the work queue */
	}

	status = usb_ep_queue(dev->gadget->ep0, preq->req, GFP_ATOMIC);
	DBG(15, "device_setup_complete ep_queue --> %d\n", status);
	if (status < 0) {
		ERROR("device_setup_complete ep_queue --> %d\n", status);
		preq->req->status = 0;
		gadget_setup_complete(dev->gadget->ep0, preq->req);
	}
}

static void device_setup_out_complete(struct urb *urb) {
	struct proxy_request *preq = urb->context;

	DBG(20, "Device setup OUT complete (%d; %d)!\n",
	    urb->status, urb->actual_length);

	preq->state = PREQ_STATE_READY;
	kfree(preq->ctrl);
	gadget_handle_setup();
}

/* Device IN endpoint, got some data from the device,
 * forward data to the host. */
static void device_epin_irq(struct urb *urb)
{
	struct proxy_request *preq = urb->context;
	int status;
	int i;

	preq->state = PREQ_STATE_BUSY;

	switch (urb->status) {
	case 0:			/* success */
		break;
	case -ECONNRESET:	/* unlink */
	case -ENOENT:
	case -ESHUTDOWN:
		ERROR("(%p) EP-IN error: status %d, not resubmitting\n",
			preq, urb->status);
		preq->state = PREQ_STATE_READY;
		return;
	/* -EPIPE:  should clear the halt */
	default:		/* error */
		ERROR("EP-IN error: status %d, resubmitting\n", urb->status);
		goto resubmit;
	}

	DBG(30, "Got an EP-IN urb back (%d)!\n", urb->actual_length);

	if (usb_endpoint_xfer_isoc(preq->ep->desc)) {
		if (!strcmp(preq->ep->gadget_ep->name, "ep1in")) {
			int iii = 0;

			DBG(5, "(%p - %s) device_epin_irq (%d (a=%d), n=%d, status=%d)\n",
				preq, preq->ep->gadget_ep->name,
				urb->transfer_buffer_length,
				urb->actual_length, urb->number_of_packets,
				urb->status);

			for (i = 0; i < urb->number_of_packets; i++) {
				if (urb->iso_frame_desc[i].actual_length > 0 ||
					urb->iso_frame_desc[i].status != 0) {
					DBG(2, "P%d: off=%d, len=%d, act=%d, status=%d (%08x)\n", i,
						urb->iso_frame_desc[i].offset,
						urb->iso_frame_desc[i].length,
						urb->iso_frame_desc[i].actual_length,
						urb->iso_frame_desc[i].status,
						le32_to_cpu(((int*)urb->transfer_buffer)[urb->iso_frame_desc[i].offset/4]));
					iii = 1;
				}

				urb->iso_frame_desc[i].offset = i*768;
				urb->iso_frame_desc[i].length = 768;
				urb->iso_frame_desc[i].status = 0;
			}

			//memcpy(preq->req->buf, urb->transfer_buffer, urb->actual_length);
			//((int*)preq->req->buf)[0] = 0xdeadbeef;

			if (iii) {
				DBG(2, "(%p - %s) device_epin_irq (%d (a=%d), n=%d, status=%d)\n",
					preq, preq->ep->gadget_ep->name,
					urb->transfer_buffer_length,
					urb->actual_length, urb->number_of_packets,
					urb->status);
			}
		}

		if (urb->actual_length == 0) {
			/* FIXME: Useless to submit request (maybe?) */
			preq->req->status = 0;
			preq->req->length = 0;
			gadget_epin_complete(preq->ep->gadget_ep, preq->req);
			return;
		}

	}

	//memcpy(preq->req->buf, urb->transfer_buffer, urb->actual_length);
	//preq->req->buf = urb->transfer_buffer;

	preq->req->length = urb->actual_length;
	preq->req->zero = 0;
	
	status = usb_ep_queue(preq->ep->gadget_ep, preq->req, GFP_ATOMIC);
	DBG(30, "device_epin_irq ep_queue --> %d\n", status);
	if (status < 0) {
		ERROR("device_epin_irq ep_queue --> %d\n", status);
		preq->req->status = status;
		gadget_epin_complete(preq->ep->gadget_ep, preq->req);
	}
#if 0
else if (usb_endpoint_xfer_isoc(preq->ep->desc)) {
		/* No callback for ISOC packets. */
		preq->req->status = 0;
		gadget_epin_complete(preq->ep->gadget_ep, preq->req);
	}
#endif

	return;

resubmit:
	; /* Resubmission is done in proxy_ep1_complete normally */
	status = usb_submit_urb (urb, GFP_ATOMIC);
	if (status) {
		ERROR("can't resubmit urb, status %d\n", status);
		preq->state = PREQ_STATE_READY;
		return;
	}

	preq->state = PREQ_STATE_SUBMITTED;
}

/* Device OUT endpoint, data was sent properly to the device,
 * resubmit request, to receive next packet from the host. */
static void device_epout_irq(struct urb *urb) {
	struct proxy_request *preq = urb->context;

	int status;

	DBG(25, "EP-OUT IRQ callback\n");

	preq->state = PREQ_STATE_SUBMITTED;

	status = usb_ep_queue(preq->ep->gadget_ep, preq->req, GFP_ATOMIC);
	DBG(25, "epout_queue --> %d\n", status);
	if (status < 0) {
		ERROR("device_epout_irq ep_queue --> %d\n", status);
		preq->state = PREQ_STATE_READY;
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
	return status;
}

static void device_disconnect(struct usb_device *udev) {
	struct proxy_dev	*dev;
	int status, i;
	dev = &usb_proxy_gadget;

	INFO("device_disconnect\n");

	kfree(udev->config);
	udev->config = NULL;
	/* FIXME */
/*
	if (dev->ep0.preq) {
		usb_kill_urb(dev->ep0.preq->urb);
		proxy_req_free(dev->ep0.preq);
		dev->ep0.preq = NULL;
		}*/

	for (i = 0; i < USB_MAXENDPOINT; i++) {
		if (dev->eps[i]) {
			proxy_disable_ep(dev->eps[i], true);
			kfree(dev->eps[i]);
			dev->eps[i] = NULL;
		}
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
