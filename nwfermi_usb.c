// SPDX-License-Identifier: GPL-2.0-only
/*
 * NextWindow Fermi optical touchscreen driver
 *
 * Fermi controllers do not report touch positions: interface 0 streams raw
 * camera waveforms over a bulk-in endpoint.  The touch engine in
 * nwfermi_engine.c finds finger shadows in that stream and triangulates them
 * using the lens and calibration data the controller sends along with it.
 * Contacts are reported through a multi-touch protocol B input device
 * (INPUT_PROP_DIRECT) parented to the USB interface, as libinput expects.
 *
 * /dev/nwfermiN (USB class device, minor base 192) exposes the raw stream
 * for diagnostics.
 *
 * The controller wants a periodic keep-alive output report on its HID
 * interface, formerly sent from userspace by the vendor's "fwprod" tool.
 *
 * Copyright (C) 2026
 */

#include <linux/cleanup.h>
#include <linux/device/devres.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/kref.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/usb.h>
#include <linux/usb/input.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "nwfermi_engine.h"

#define NWFERMI_MINOR_BASE	192
#define NWFERMI_NUM_URBS	4
#define NWFERMI_URB_SIZE	4096
#define NWFERMI_FIFO_SIZE	SZ_64K
#define NWFERMI_CHUNK		4096

#define NWFERMI_MAX_CONTACTS	NWF_MAX_CONTACTS
#define NWFERMI_MAX_COORD	NWF_MAX_COORD

#define NWFERMI_HID_IFNUM	1
#define NWFERMI_HID_REPORT_LEN	64
#define NWFERMI_CTRL_TIMEOUT_MS	1000
#define NWFERMI_RECOVER_DELAY_MS 100

struct nwfermi_keepalive {
	const u8 *data;
	size_t len;
};

static const u8 nwfermi_keepalive_1846_data[] = { 0x43, 0x02, 0xf3, 0x00 };
static const u8 nwfermi_keepalive_1878_data[] = { 0x43, 0x02, 0xf0, 0x01 };

static const struct nwfermi_keepalive nwfermi_keepalive_1846 = {
	.data = nwfermi_keepalive_1846_data,
	.len = sizeof(nwfermi_keepalive_1846_data),
};

static const struct nwfermi_keepalive nwfermi_keepalive_1878 = {
	.data = nwfermi_keepalive_1878_data,
	.len = sizeof(nwfermi_keepalive_1878_data),
};

static unsigned int keepalive_interval = 300;
module_param(keepalive_interval, uint, 0444);
MODULE_PARM_DESC(keepalive_interval,
		 "Seconds between firmware keep-alive reports (0 = disabled, default 300)");

struct nwfermi {
	struct usb_device *udev;
	struct usb_interface *intf;	/* NULL once disconnected */
	struct input_dev *input;
	struct kref kref;

	/* Serialises users, streaming state, reset and PM. */
	struct mutex io_mutex;
	unsigned int users;		/* input device + char device opens */
	unsigned int readers;		/* char device opens */
	bool streaming;
	bool disconnected;

	/* Raw stream copy for /dev/nwfermiN readers. */
	struct mutex read_mutex;
	spinlock_t fifo_lock;		/* fifo producers vs. each other */
	struct kfifo fifo;
	wait_queue_head_t wait;

	/* Raw stream for the touch engine. */
	spinlock_t engine_fifo_lock;	/* engine_fifo producers */
	struct kfifo engine_fifo;
	struct mutex engine_mutex;	/* engine state */
	struct nwf_engine *engine;
	u8 *chunk;
	struct work_struct engine_work;
	struct delayed_work idle_work;

	unsigned int bulk_in_pipe;
	struct usb_anchor anchor;
	struct urb *urbs[NWFERMI_NUM_URBS];

	struct delayed_work recover_work;
	unsigned long flags;
#define NWFERMI_STALLED		0

	const struct nwfermi_keepalive *keepalive;
	unsigned int keepalive_pipe;
	struct delayed_work keepalive_work;

	char phys[64];
};

static struct usb_driver nwfermi_driver;

static void nwfermi_delete(struct kref *kref)
{
	struct nwfermi *nw = container_of(kref, struct nwfermi, kref);
	int i;

	for (i = 0; i < NWFERMI_NUM_URBS; i++) {
		if (nw->urbs[i])
			kfree(nw->urbs[i]->transfer_buffer);
		usb_free_urb(nw->urbs[i]);
	}
	kfifo_free(&nw->fifo);
	kfifo_free(&nw->engine_fifo);
	kfree(nw->chunk);
	kvfree(nw->engine);
	usb_put_dev(nw->udev);
	kfree(nw);
}

static void nwfermi_put(void *data)
{
	struct nwfermi *nw = data;

	kref_put(&nw->kref, nwfermi_delete);
}

/* ---------------------------------------------------------------------- */
/* Raw waveform stream                                                     */

static void nwfermi_push(struct nwfermi *nw, struct kfifo *fifo,
			 spinlock_t *lock, const void *buf, unsigned int len)
{
	unsigned int copied;

	scoped_guard(spinlock_irqsave, lock)
		copied = kfifo_in(fifo, buf, len);
	if (copied < len)
		dev_warn_ratelimited(&nw->udev->dev,
				     "stream buffer full, data dropped\n");
}

static void nwfermi_bulk_complete(struct urb *urb)
{
	struct nwfermi *nw = urb->context;
	unsigned int len = urb->actual_length;
	int error;

	switch (urb->status) {
	case 0:
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		return;		/* killed or device gone */
	case -EPIPE:
		set_bit(NWFERMI_STALLED, &nw->flags);
		fallthrough;
	default:
		dev_dbg(&nw->udev->dev, "bulk-in error %d, recovering\n",
			urb->status);
		schedule_delayed_work(&nw->recover_work,
				      msecs_to_jiffies(NWFERMI_RECOVER_DELAY_MS));
		return;
	}

	if (len) {
		if (READ_ONCE(nw->readers)) {
			nwfermi_push(nw, &nw->fifo, &nw->fifo_lock,
				     urb->transfer_buffer, len);
			wake_up_interruptible(&nw->wait);
		}
		nwfermi_push(nw, &nw->engine_fifo, &nw->engine_fifo_lock,
			     urb->transfer_buffer, len);
		queue_work(system_highpri_wq, &nw->engine_work);
	}

	usb_anchor_urb(urb, &nw->anchor);
	error = usb_submit_urb(urb, GFP_ATOMIC);
	if (error) {
		usb_unanchor_urb(urb);
		if (error != -EPERM && error != -ENODEV)
			dev_err(&nw->udev->dev,
				"failed to resubmit bulk urb: %d\n", error);
	}
}

/* Called with io_mutex held. */
static int nwfermi_start_stream(struct nwfermi *nw)
{
	int error;
	int i;

	if (nw->streaming)
		return 0;

	for (i = 0; i < NWFERMI_NUM_URBS; i++) {
		usb_anchor_urb(nw->urbs[i], &nw->anchor);
		error = usb_submit_urb(nw->urbs[i], GFP_KERNEL);
		if (error) {
			usb_unanchor_urb(nw->urbs[i]);
			usb_kill_anchored_urbs(&nw->anchor);
			dev_err(&nw->udev->dev,
				"failed to submit bulk urb: %d\n", error);
			return error;
		}
	}

	nw->streaming = true;
	return 0;
}

/* Called with io_mutex held. */
static void nwfermi_stop_stream(struct nwfermi *nw)
{
	nw->streaming = false;
	usb_kill_anchored_urbs(&nw->anchor);
}

/* Called with io_mutex held. */
static int nwfermi_get_user(struct nwfermi *nw)
{
	int error;

	if (nw->disconnected)
		return -ENODEV;

	error = nwfermi_start_stream(nw);
	if (!error)
		nw->users++;
	return error;
}

/* Called with io_mutex held. */
static void nwfermi_put_user(struct nwfermi *nw)
{
	if (!--nw->users && !nw->disconnected)
		nwfermi_stop_stream(nw);
	/* Async: a synchronous suspend would retake io_mutex. */
	if (nw->intf)
		usb_autopm_put_interface_async(nw->intf);
}

static void nwfermi_recover_work(struct work_struct *work)
{
	struct nwfermi *nw = container_of(to_delayed_work(work),
					  struct nwfermi, recover_work);
	int error;

	guard(mutex)(&nw->io_mutex);

	if (nw->disconnected || !nw->streaming)
		return;

	nwfermi_stop_stream(nw);

	if (test_and_clear_bit(NWFERMI_STALLED, &nw->flags)) {
		error = usb_clear_halt(nw->udev, nw->bulk_in_pipe);
		if (error) {
			dev_err(&nw->udev->dev,
				"failed to clear bulk-in halt: %d\n", error);
			return;
		}
	}

	nwfermi_start_stream(nw);
}

/* ---------------------------------------------------------------------- */
/* Firmware keep-alive (replaces the userspace "fwprod" utility)           */

static void nwfermi_schedule_keepalive(struct nwfermi *nw, unsigned long delay)
{
	unsigned int interval = READ_ONCE(keepalive_interval);

	if (nw->keepalive && interval)
		schedule_delayed_work(&nw->keepalive_work,
				      delay ?: secs_to_jiffies(interval));
}

static void nwfermi_keepalive_work(struct work_struct *work)
{
	struct nwfermi *nw = container_of(to_delayed_work(work),
					  struct nwfermi, keepalive_work);
	struct usb_interface *intf = nw->intf;
	int actual;
	int error;

	u8 *buf __free(kfree) = kzalloc(NWFERMI_HID_REPORT_LEN, GFP_KERNEL);
	if (!buf)
		goto out;

	memcpy(buf, nw->keepalive->data, nw->keepalive->len);

	error = usb_autopm_get_interface(intf);
	if (error)
		goto out;

	error = usb_interrupt_msg(nw->udev, nw->keepalive_pipe, buf,
				  NWFERMI_HID_REPORT_LEN, &actual,
				  NWFERMI_CTRL_TIMEOUT_MS);
	if (error)
		dev_warn(&intf->dev, "keep-alive report failed: %d\n", error);

	/* A synchronous put could suspend us, and suspend() cancels this work. */
	usb_autopm_put_interface_async(intf);
out:
	nwfermi_schedule_keepalive(nw, 0);
}

static void nwfermi_init_keepalive(struct nwfermi *nw)
{
	struct usb_endpoint_descriptor *ep;
	struct usb_interface *hid_intf;

	if (!nw->keepalive)
		return;

	/*
	 * The keep-alive is an output report on the vendor HID interface.
	 * usbhid normally owns that interface; the report goes straight to its
	 * interrupt-out endpoint, exactly as hidraw would have sent it.
	 */
	hid_intf = usb_ifnum_to_if(nw->udev, NWFERMI_HID_IFNUM);
	if (!hid_intf ||
	    usb_find_int_out_endpoint(hid_intf->cur_altsetting, &ep)) {
		dev_dbg(&nw->intf->dev, "no HID out endpoint, keep-alive off\n");
		nw->keepalive = NULL;
		return;
	}

	nw->keepalive_pipe = usb_sndintpipe(nw->udev, usb_endpoint_num(ep));
	nwfermi_schedule_keepalive(nw, 1);
}

/* ---------------------------------------------------------------------- */
/* Touch engine                                                            */

static void nwfermi_engine_report(void *ctx, const struct nwf_contact *c,
				  unsigned int n)
{
	struct nwfermi *nw = ctx;
	struct input_dev *input = nw->input;
	unsigned int i;
	int slot;

	for (i = 0; i < n; i++) {
		slot = input_mt_get_slot_by_key(input, c[i].id);
		if (slot < 0)
			continue;

		input_mt_slot(input, slot);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
		input_report_abs(input, ABS_MT_POSITION_X, c[i].x);
		input_report_abs(input, ABS_MT_POSITION_Y, c[i].y);
	}

	input_mt_sync_frame(input);
	input_sync(input);

	/* The controller stops streaming when untouched: time out lifts. */
	if (n)
		mod_delayed_work(system_highpri_wq, &nw->idle_work,
				 msecs_to_jiffies(NWF_IDLE_TIMEOUT_MS));
}

static void nwfermi_engine_work(struct work_struct *work)
{
	struct nwfermi *nw = container_of(work, struct nwfermi, engine_work);
	unsigned int len;

	guard(mutex)(&nw->engine_mutex);

	while ((len = kfifo_out_spinlocked(&nw->engine_fifo, nw->chunk,
					   NWFERMI_CHUNK,
					   &nw->engine_fifo_lock)))
		nwf_engine_feed(nw->engine, nw->chunk, len);
}

static void nwfermi_idle_work(struct work_struct *work)
{
	struct nwfermi *nw = container_of(to_delayed_work(work),
					  struct nwfermi, idle_work);

	guard(mutex)(&nw->engine_mutex);
	nwf_engine_idle(nw->engine);
}

static int nwfermi_init_engine(struct nwfermi *nw)
{
	int error;

	nw->engine = kvzalloc_obj(*nw->engine);
	nw->chunk = kmalloc(NWFERMI_CHUNK, GFP_KERNEL);
	if (!nw->engine || !nw->chunk)
		return -ENOMEM;

	error = kfifo_alloc(&nw->engine_fifo, NWFERMI_FIFO_SIZE, GFP_KERNEL);
	if (error)
		return error;

	nwf_engine_init(nw->engine, nwfermi_engine_report, nw);
	return 0;
}

/* ---------------------------------------------------------------------- */
/* Character device                                                        */

static int nwfermi_open(struct inode *inode, struct file *file)
{
	struct usb_interface *intf;
	struct nwfermi *nw;
	int error;

	intf = usb_find_interface(&nwfermi_driver, iminor(inode));
	if (!intf)
		return -ENODEV;

	nw = usb_get_intfdata(intf);
	if (!nw)
		return -ENODEV;

	/* Resume before taking io_mutex: resume() takes it too. */
	error = usb_autopm_get_interface(intf);
	if (error)
		return error;

	scoped_guard(mutex, &nw->io_mutex) {
		error = nwfermi_get_user(nw);
		if (!error) {
			WRITE_ONCE(nw->readers, nw->readers + 1);
			kref_get(&nw->kref);
		}
	}

	if (error) {
		usb_autopm_put_interface(intf);
		return error;
	}

	file->private_data = nw;
	return 0;
}

static int nwfermi_release(struct inode *inode, struct file *file)
{
	struct nwfermi *nw = file->private_data;

	scoped_guard(mutex, &nw->io_mutex) {
		WRITE_ONCE(nw->readers, nw->readers - 1);
		nwfermi_put_user(nw);
	}

	kref_put(&nw->kref, nwfermi_delete);
	return 0;
}

static ssize_t nwfermi_read(struct file *file, char __user *buf, size_t count,
			    loff_t *ppos)
{
	struct nwfermi *nw = file->private_data;
	unsigned int copied;
	int error;

	if (!count)
		return 0;

	for (;;) {
		scoped_guard(mutex, &nw->read_mutex) {
			if (READ_ONCE(nw->disconnected))
				return -ENODEV;

			if (!kfifo_is_empty(&nw->fifo)) {
				error = kfifo_to_user(&nw->fifo, buf, count,
						      &copied);
				return error ?: copied;
			}
		}

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;

		error = wait_event_interruptible(nw->wait,
						 !kfifo_is_empty(&nw->fifo) ||
						 READ_ONCE(nw->disconnected));
		if (error)
			return error;
	}
}

static __poll_t nwfermi_poll(struct file *file, poll_table *wait)
{
	struct nwfermi *nw = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &nw->wait, wait);

	if (READ_ONCE(nw->disconnected))
		return EPOLLERR | EPOLLHUP;

	if (!kfifo_is_empty(&nw->fifo))
		mask |= EPOLLIN | EPOLLRDNORM;

	return mask;
}

static const struct file_operations nwfermi_fops = {
	.owner		= THIS_MODULE,
	.open		= nwfermi_open,
	.release	= nwfermi_release,
	.read		= nwfermi_read,
	.poll		= nwfermi_poll,
	.llseek		= noop_llseek,
};

static struct usb_class_driver nwfermi_class = {
	.name		= "nwfermi%d",
	.fops		= &nwfermi_fops,
	.minor_base	= NWFERMI_MINOR_BASE,
};

/* ---------------------------------------------------------------------- */
/* Input device                                                            */

static int nwfermi_input_open(struct input_dev *input)
{
	struct nwfermi *nw = input_get_drvdata(input);
	int error;

	error = usb_autopm_get_interface(nw->intf);
	if (error)
		return error;

	scoped_guard(mutex, &nw->io_mutex)
		error = nwfermi_get_user(nw);

	if (error)
		usb_autopm_put_interface(nw->intf);
	return error;
}

static void nwfermi_input_close(struct input_dev *input)
{
	struct nwfermi *nw = input_get_drvdata(input);

	scoped_guard(mutex, &nw->io_mutex)
		nwfermi_put_user(nw);
}

static int nwfermi_setup_input(struct nwfermi *nw)
{
	struct input_dev *input;
	int error;

	/* The input device can outlive disconnect(); pin our state for it. */
	kref_get(&nw->kref);
	error = devm_add_action_or_reset(&nw->intf->dev, nwfermi_put, nw);
	if (error)
		return error;

	input = devm_input_allocate_device(&nw->intf->dev);
	if (!input)
		return -ENOMEM;

	usb_make_path(nw->udev, nw->phys, sizeof(nw->phys));
	strlcat(nw->phys, "/input0", sizeof(nw->phys));

	input->name = "Nextwindow Fermi Touchscreen";
	input->phys = nw->phys;
	usb_to_input_id(nw->udev, &input->id);
	input->open = nwfermi_input_open;
	input->close = nwfermi_input_close;
	input_set_drvdata(input, nw);

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, NWFERMI_MAX_COORD,
			     0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, NWFERMI_MAX_COORD,
			     0, 0);

	error = input_mt_init_slots(input, NWFERMI_MAX_CONTACTS,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error)
		return error;

	nw->input = input;

	error = input_register_device(input);
	if (error)
		return error;

	return 0;
}

/* ---------------------------------------------------------------------- */
/* USB driver                                                              */

static int nwfermi_alloc_urbs(struct nwfermi *nw)
{
	struct urb *urb;
	void *buf;
	int i;

	for (i = 0; i < NWFERMI_NUM_URBS; i++) {
		urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!urb)
			return -ENOMEM;
		nw->urbs[i] = urb;

		buf = kmalloc(NWFERMI_URB_SIZE, GFP_KERNEL);
		if (!buf)
			return -ENOMEM;

		usb_fill_bulk_urb(urb, nw->udev, nw->bulk_in_pipe, buf,
				  NWFERMI_URB_SIZE, nwfermi_bulk_complete, nw);
	}

	return 0;
}

static int nwfermi_probe(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	struct usb_endpoint_descriptor *bulk_in;
	struct nwfermi *nw;
	int error;

	/* The HID control interface is left to usbhid. */
	if (usb_find_bulk_in_endpoint(intf->cur_altsetting, &bulk_in))
		return -ENODEV;

	nw = kzalloc_obj(*nw);
	if (!nw)
		return -ENOMEM;

	kref_init(&nw->kref);
	mutex_init(&nw->io_mutex);
	mutex_init(&nw->read_mutex);
	mutex_init(&nw->engine_mutex);
	spin_lock_init(&nw->fifo_lock);
	spin_lock_init(&nw->engine_fifo_lock);
	init_waitqueue_head(&nw->wait);
	init_usb_anchor(&nw->anchor);
	INIT_WORK(&nw->engine_work, nwfermi_engine_work);
	INIT_DELAYED_WORK(&nw->idle_work, nwfermi_idle_work);
	INIT_DELAYED_WORK(&nw->recover_work, nwfermi_recover_work);
	INIT_DELAYED_WORK(&nw->keepalive_work, nwfermi_keepalive_work);

	nw->udev = usb_get_dev(interface_to_usbdev(intf));
	nw->intf = intf;
	nw->keepalive = (const struct nwfermi_keepalive *)id->driver_info;
	nw->bulk_in_pipe = usb_rcvbulkpipe(nw->udev,
					   usb_endpoint_num(bulk_in));

	error = kfifo_alloc(&nw->fifo, NWFERMI_FIFO_SIZE, GFP_KERNEL);
	if (error)
		goto err_put;

	error = nwfermi_init_engine(nw);
	if (error)
		goto err_put;

	error = nwfermi_alloc_urbs(nw);
	if (error)
		goto err_put;

	usb_set_intfdata(intf, nw);

	error = nwfermi_setup_input(nw);
	if (error) {
		dev_err(&intf->dev, "failed to register input device: %d\n",
			error);
		goto err_clear;
	}

	error = usb_register_dev(intf, &nwfermi_class);
	if (error) {
		dev_err(&intf->dev, "unable to get a minor: %d\n", error);
		goto err_clear;
	}

	nwfermi_init_keepalive(nw);

	dev_info(&intf->dev, "NextWindow Fermi touchscreen on nwfermi%d\n",
		 intf->minor - nwfermi_class.minor_base);
	return 0;

err_clear:
	usb_set_intfdata(intf, NULL);
err_put:
	kref_put(&nw->kref, nwfermi_delete);
	return error;
}

static void nwfermi_disconnect(struct usb_interface *intf)
{
	struct nwfermi *nw = usb_get_intfdata(intf);

	usb_deregister_dev(intf, &nwfermi_class);

	disable_delayed_work_sync(&nw->keepalive_work);
	disable_delayed_work_sync(&nw->recover_work);

	scoped_guard(mutex, &nw->io_mutex) {
		WRITE_ONCE(nw->disconnected, true);
		nwfermi_stop_stream(nw);
		nw->intf = NULL;
	}

	disable_work_sync(&nw->engine_work);
	disable_delayed_work_sync(&nw->idle_work);

	wake_up_interruptible_all(&nw->wait);
	usb_set_intfdata(intf, NULL);

	/* The devm-managed input device is unregistered after this returns. */
	kref_put(&nw->kref, nwfermi_delete);
}

static int nwfermi_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct nwfermi *nw = usb_get_intfdata(intf);

	cancel_delayed_work_sync(&nw->keepalive_work);
	cancel_delayed_work_sync(&nw->recover_work);

	scoped_guard(mutex, &nw->io_mutex) {
		if (nw->streaming) {
			usb_kill_anchored_urbs(&nw->anchor);
			nw->streaming = false;
		}
	}

	cancel_work_sync(&nw->engine_work);
	cancel_delayed_work_sync(&nw->idle_work);
	scoped_guard(mutex, &nw->engine_mutex)
		nwf_engine_idle(nw->engine);	/* no stuck contacts */

	return 0;
}

static int nwfermi_resume(struct usb_interface *intf)
{
	struct nwfermi *nw = usb_get_intfdata(intf);
	int error = 0;

	scoped_guard(mutex, &nw->io_mutex) {
		if (nw->users)
			error = nwfermi_start_stream(nw);
	}

	nwfermi_schedule_keepalive(nw, 1);
	return error;
}

static int nwfermi_pre_reset(struct usb_interface *intf)
{
	struct nwfermi *nw = usb_get_intfdata(intf);

	cancel_delayed_work_sync(&nw->recover_work);

	/* Held until post_reset(). */
	mutex_lock(&nw->io_mutex);
	if (nw->streaming) {
		usb_kill_anchored_urbs(&nw->anchor);
		nw->streaming = false;
	}

	return 0;
}

static int nwfermi_post_reset(struct usb_interface *intf)
{
	struct nwfermi *nw = usb_get_intfdata(intf);
	int error = 0;

	if (nw->users)
		error = nwfermi_start_stream(nw);
	mutex_unlock(&nw->io_mutex);

	return error;
}

static const struct usb_device_id nwfermi_ids[] = {
	{ USB_DEVICE(0x0b8c, 0x0069) },
	{ USB_DEVICE(0x1926, 0x0006) },
	{ USB_DEVICE(0x1926, 0x0009) },
	{ USB_DEVICE(0x1926, 0x0064) },
	{ USB_DEVICE(0x1926, 0x0065) },
	{ USB_DEVICE(0x1926, 0x0066) },
	{ USB_DEVICE(0x1926, 0x0067) },
	{ USB_DEVICE(0x1926, 0x0068) },
	{ USB_DEVICE(0x1926, 0x0069) },
	{ USB_DEVICE(0x1926, 0x006a) },
	{ USB_DEVICE(0x1926, 0x006b) },
	{ USB_DEVICE(0x1926, 0x006c) },
	{ USB_DEVICE(0x1926, 0x006d) },
	{ USB_DEVICE(0x1926, 0x006e) },
	{ USB_DEVICE(0x1926, 0x006f) },
	{ USB_DEVICE(0x1926, 0x0070) },
	{ USB_DEVICE(0x1926, 0x0071) },
	{ USB_DEVICE(0x1926, 0x0072) },
	{ USB_DEVICE(0x1926, 0x0073) },
	{ USB_DEVICE(0x1926, 0x0074) },
	{ USB_DEVICE(0x1926, 0x0075) },
	{ USB_DEVICE(0x1926, 0x0076) },
	{ USB_DEVICE(0x1926, 0x0077) },
	{ USB_DEVICE(0x1926, 0x0078) },
	{ USB_DEVICE(0x1926, 0x0079) },
	{ USB_DEVICE(0x1926, 0x007a) },
	{ USB_DEVICE(0x1926, 0x007b) },
	{ USB_DEVICE(0x1926, 0x007c) },
	{ USB_DEVICE(0x1926, 0x007d) },
	{ USB_DEVICE(0x1926, 0x007e) },
	{ USB_DEVICE(0x1926, 0x007f) },
	{ USB_DEVICE(0x1926, 0x0080) },
	{ USB_DEVICE(0x1926, 0x0081) },
	{ USB_DEVICE(0x1926, 0x0082) },
	{ USB_DEVICE(0x1926, 0x0083) },
	{ USB_DEVICE(0x1926, 0x0084) },
	{ USB_DEVICE(0x1926, 0x0085) },
	{ USB_DEVICE(0x1926, 0x0086) },
	{ USB_DEVICE(0x1926, 0x0087) },
	{ USB_DEVICE(0x1926, 0x0088) },
	{ USB_DEVICE(0x1926, 0x0089) },
	{ USB_DEVICE(0x1926, 0x008a) },
	{ USB_DEVICE(0x1926, 0x008b) },
	{ USB_DEVICE(0x1926, 0x008c) },
	{ USB_DEVICE(0x1926, 0x008d) },
	{ USB_DEVICE(0x1926, 0x008e) },
	{ USB_DEVICE(0x1926, 0x008f) },
	{ USB_DEVICE(0x1926, 0x0090) },
	{ USB_DEVICE(0x1926, 0x0091) },
	{ USB_DEVICE(0x1926, 0x0092) },
	{ USB_DEVICE(0x1926, 0x0093) },
	{ USB_DEVICE(0x1926, 0x0094) },
	{ USB_DEVICE(0x1926, 0x0095) },
	{ USB_DEVICE(0x1926, 0x0096) },
	{ USB_DEVICE(0x1926, 0x0097) },
	{ USB_DEVICE(0x1926, 0x0098) },
	{ USB_DEVICE(0x1926, 0x0099) },
	{ USB_DEVICE(0x1926, 0x009a) },
	{ USB_DEVICE(0x1926, 0x09c4) },
	{ USB_DEVICE(0x1926, 0x0bb8) },
	{ USB_DEVICE(0x1926, 0x0bb9) },
	{ USB_DEVICE(0x1926, 0x0bba) },
	{ USB_DEVICE(0x1926, 0x0bbb) },
	{ USB_DEVICE(0x1926, 0x0bbc) },
	{ USB_DEVICE(0x1926, 0x0bbd) },
	{ USB_DEVICE(0x1926, 0x0bbe) },
	{ USB_DEVICE(0x1926, 0x0bbf) },
	{ USB_DEVICE(0x1926, 0x0bc0) },
	{ USB_DEVICE(0x1926, 0x0bc1) },
	{ USB_DEVICE(0x1926, 0x0bc2) },
	{ USB_DEVICE(0x1926, 0x0bc3) },
	{ USB_DEVICE(0x1926, 0x0bc4) },
	{ USB_DEVICE(0x1926, 0x0bc5) },
	{ USB_DEVICE(0x1926, 0x0bc6) },
	{ USB_DEVICE(0x1926, 0x0bc7) },
	{ USB_DEVICE(0x1926, 0x0bc8) },
	{ USB_DEVICE(0x1926, 0x0bc9) },
	{ USB_DEVICE(0x1926, 0x0bca) },
	{ USB_DEVICE(0x1926, 0x0bcb) },
	{ USB_DEVICE(0x1926, 0x0bcc) },
	{ USB_DEVICE(0x1926, 0x0bcd) },
	{ USB_DEVICE(0x1926, 0x0bce) },
	{ USB_DEVICE(0x1926, 0x0bd2) },
	{ USB_DEVICE(0x1926, 0x0c1c) },
	{ USB_DEVICE(0x1926, 0x0c80) },
	{ USB_DEVICE(0x1926, 0x0c81) },
	{ USB_DEVICE(0x1926, 0x0c82) },
	{ USB_DEVICE(0x1926, 0x0c83) },
	{ USB_DEVICE(0x1926, 0x0c84) },
	{ USB_DEVICE(0x1926, 0x0c85) },
	{ USB_DEVICE(0x1926, 0x0c87) },
	{ USB_DEVICE(0x1926, 0x0c88) },
	{ USB_DEVICE(0x1926, 0x0c89) },
	{ USB_DEVICE(0x1926, 0x0c8a) },
	{ USB_DEVICE(0x1926, 0x0c8b) },
	{ USB_DEVICE(0x1926, 0x0c8c) },
	{ USB_DEVICE(0x1926, 0x0d49) },
	{ USB_DEVICE(0x1926, 0x0d4a) },
	{ USB_DEVICE(0x1926, 0x0dac) },
	{ USB_DEVICE(0x1926, 0x0dad) },
	{ USB_DEVICE(0x1926, 0x0dae) },
	{ USB_DEVICE(0x1926, 0x0daf) },
	{ USB_DEVICE(0x1926, 0x0db0) },
	{ USB_DEVICE(0x1926, 0x0db1) },
	{ USB_DEVICE(0x1926, 0x0db2) },
	{ USB_DEVICE(0x1926, 0x0db3) },
	{ USB_DEVICE(0x1926, 0x0db4) },
	{ USB_DEVICE(0x1926, 0x0db5) },
	{ USB_DEVICE(0x1926, 0x0db6) },
	{ USB_DEVICE(0x1926, 0x0db7) },
	{ USB_DEVICE(0x1926, 0x0dbc) },
	{ USB_DEVICE(0x1926, 0x0dbd) },
	{ USB_DEVICE(0x1926, 0x0dbe) },
	{ USB_DEVICE(0x1926, 0x0dbf) },
	{ USB_DEVICE(0x1926, 0x0dc0) },
	{ USB_DEVICE(0x1926, 0x0dc1) },
	{ USB_DEVICE(0x1926, 0x0dc2) },
	{ USB_DEVICE(0x1926, 0x0dc5) },
	{ USB_DEVICE(0x1926, 0x0e11) },
	{ USB_DEVICE(0x1926, 0x0e12) },
	{ USB_DEVICE(0x1926, 0x0e13) },
	{ USB_DEVICE(0x1926, 0x0e14) },
	{ USB_DEVICE(0x1926, 0x0e15) },
	{ USB_DEVICE(0x1926, 0x0e16) },
	{ USB_DEVICE(0x1926, 0x0e17) },
	{ USB_DEVICE(0x1926, 0x0ed9) },
	{ USB_DEVICE(0x1926, 0x0f3d) },
	{ USB_DEVICE(0x1926, 0x0f3e) },
	{ USB_DEVICE(0x1926, 0x183e) },
	{ USB_DEVICE(0x1926, 0x183f) },
	{ USB_DEVICE(0x1926, 0x1840) },
	{ USB_DEVICE(0x1926, 0x1841) },
	{ USB_DEVICE(0x1926, 0x1842) },
	{ USB_DEVICE(0x1926, 0x1843) },
	{ USB_DEVICE(0x1926, 0x1844) },
	{ USB_DEVICE(0x1926, 0x1845) },
	{ USB_DEVICE(0x1926, 0x1846), .driver_info = (kernel_ulong_t)&nwfermi_keepalive_1846 },
	{ USB_DEVICE(0x1926, 0x1847) },
	{ USB_DEVICE(0x1926, 0x1848) },
	{ USB_DEVICE(0x1926, 0x1878), .driver_info = (kernel_ulong_t)&nwfermi_keepalive_1878 },
	{ }
};
MODULE_DEVICE_TABLE(usb, nwfermi_ids);

static struct usb_driver nwfermi_driver = {
	.name			= "nwfermi",
	.probe			= nwfermi_probe,
	.disconnect		= nwfermi_disconnect,
	.suspend		= nwfermi_suspend,
	.resume			= nwfermi_resume,
	.reset_resume		= nwfermi_resume,
	.pre_reset		= nwfermi_pre_reset,
	.post_reset		= nwfermi_post_reset,
	.id_table		= nwfermi_ids,
	.supports_autosuspend	= 1,
};
module_usb_driver(nwfermi_driver);

MODULE_DESCRIPTION("NextWindow Fermi optical touchscreen driver");
MODULE_LICENSE("GPL");
