/*
 * tps61059.c - tps61059 Flash driver
 *
 * Copyright (C) 2009 Texas Instruments.
 *
 * Contributors:
 * 	Nikolay Vladimirov <nvladimirov@mm-sol.com>
 * 	Pallavi Kulkarni <p-kulkarni@ti.com>
 * 	Sergio Aguirre <saaguirre@ti.com>
 *
 * Leverage mt9p012.c
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2. This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 */

#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/videodev2.h>

#include <media/tps61059.h>

#define CTRL_CAMERA_FLASH_STROBE		0
#define CTRL_CAMERA_FLASH_TIMEOUT		1
#define CTRL_CAMERA_FLASH_TORCH_INTENSITY	2

struct tps61059_flash {
	struct device *dev;
	struct tps61059_platform_data *pdata;
	struct v4l2_int_device *v4l2_int_device;
	u32 flash_timeout;
	u32 torch_intensity;
	struct timer_list strobe_timer;
};


static void tps61059_flash_off(unsigned long data)
{
	struct tps61059_flash *flash = (struct tps61059_flash *)data;

	flash->pdata->flash_off();
}

static int tps61059_delay_strobe(struct tps61059_flash *flash)
{
	int ret = 0;

	setup_timer(&flash->strobe_timer, tps61059_flash_off,
		    (unsigned long)flash);

	if (flash->flash_timeout) {
		ret = mod_timer(&flash->strobe_timer,
				jiffies +
				usecs_to_jiffies(flash->flash_timeout));
		if (ret)
			dev_err(flash->dev, "tps61059 mod_timer() returned %i!\n",
					ret);
	}

	return ret;
}

static int tps61059_strobe(struct v4l2_int_device *s)
{
	struct tps61059_flash *flash = s->priv;

	flash->pdata->flash_on();
	return tps61059_delay_strobe(flash);
}

static void tps61059_update_hw(struct v4l2_int_device *s)
{
	struct tps61059_flash *flash = s->priv;

	flash->pdata->s_torch_intensity(flash->torch_intensity);
}

static struct v4l2_queryctrl tps61059_ctrls[] = {
	{
		.id		= V4L2_CID_FLASH_STROBE,
		.type		= V4L2_CTRL_TYPE_BUTTON,
		.name		= "Flash strobe",
		.minimum	= 0,
		.maximum	= 0,
		.step		= 0,
		.default_value	= 0,
		.flags		= V4L2_CTRL_FLAG_UPDATE,
	},

	{
		.id		= V4L2_CID_FLASH_TIMEOUT,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Flash timeout [us]",
		.minimum	= 1,
		.maximum	= 1000000,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	},
	{
		.id		= V4L2_CID_FLASH_TORCH_INTENSITY,
		.type		= V4L2_CTRL_TYPE_INTEGER,
		.name		= "Torch intensity",
		.minimum	= 0,
		.maximum	= 1,
		.step		= 1,
		.default_value	= 0,
		.flags		= V4L2_CTRL_FLAG_SLIDER,
	},
};

/**
 * find_vctrl - Finds the requested ID in the video control structure array
 * @id: ID of control to search the video control array for
 *
 * Returns the index of the requested ID from the control structure array
 */
static int find_vctrl(int id)
{
	int i;

	if (id < V4L2_CID_BASE)
		return -EDOM;

	for (i = (ARRAY_SIZE(tps61059_ctrls) - 1); i >= 0; i--) {
		if (tps61059_ctrls[i].id == id)
			return i;
	}
	return -EINVAL;
}

/**
 * tps61059_ioctl_queryctrl - V4L2 flash interface handler for VIDIOC_QUERYCTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @qc: standard V4L2 VIDIOC_QUERYCTRL ioctl structure
 *
 * If the requested control is supported, returns the control information
 * from the video_control[] array.  Otherwise, returns -EINVAL if the
 * control is not supported.
 */
static int tps61059_ioctl_queryctrl(struct v4l2_int_device *s,
					struct v4l2_queryctrl *qc)
{
	int i;

	i = find_vctrl(qc->id);
	if (i == -EINVAL)
		qc->flags = V4L2_CTRL_FLAG_DISABLED;

	if (i < 0)
		return -EINVAL;
	*qc = tps61059_ctrls[i];
	return 0;
}

/**
 * tps61059_ioctl_g_ctrl - V4L2 tps61059 flash interface handler for VIDIOC_G_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_G_CTRL ioctl structure
 *
 * If the requested control is supported, returns the control's current
 * value from the video_control[] array.  Otherwise, returns -EINVAL
 * if the control is not supported.
 */
static int tps61059_ioctl_g_ctrl(struct v4l2_int_device *s,
			struct v4l2_control *vc)
{
	struct tps61059_flash *flash = s->priv;

	switch (vc->id) {
	case V4L2_CID_FLASH_TIMEOUT:
		vc->value = flash->flash_timeout;
		break;
	case V4L2_CID_FLASH_TORCH_INTENSITY:
		vc->value = flash->torch_intensity;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

/**
 * ioctl_s_ctrl - V4L2 tps61059 flash interface handler for VIDIOC_S_CTRL ioctl
 * @s: pointer to standard V4L2 device structure
 * @vc: standard V4L2 VIDIOC_S_CTRL ioctl structure
 *
 * If the requested control is supported, sets the control's current
 * value in HW (and updates the video_control[] array).  Otherwise,
 * returns -EINVAL if the control is not supported.
 */
static int tps61059_ioctl_s_ctrl(struct v4l2_int_device *s,
				struct v4l2_control *vc)
{
	struct tps61059_flash *flash = s->priv;
	int ctrl;
	int *value;

	switch (vc->id) {
	case V4L2_CID_FLASH_STROBE:
		return tps61059_strobe(s);

	case V4L2_CID_FLASH_TIMEOUT:
		ctrl = CTRL_CAMERA_FLASH_TIMEOUT;
		value = &flash->flash_timeout;
		break;
	case V4L2_CID_FLASH_TORCH_INTENSITY:
		ctrl = CTRL_CAMERA_FLASH_TORCH_INTENSITY;
		value = &flash->torch_intensity;
		break;
	default:
		return -EINVAL;
	}

	if (vc->value < tps61059_ctrls[ctrl].minimum)
		vc->value = tps61059_ctrls[ctrl].minimum;
	if (vc->value > tps61059_ctrls[ctrl].maximum)
		vc->value = tps61059_ctrls[ctrl].maximum;
	*value = vc->value;

	tps61059_update_hw(s);
	return 0;
}

/**
 * tps61059_ioctl_s_power - V4L2 flash interface handler for vidioc_int_s_power_num
 * @s: pointer to standard V4L2 device structure
 * @state: power state to which device is to be set
 *
 * Sets devices power state to requested state, if possible.
 */
static int tps61059_ioctl_s_power(struct v4l2_int_device *s,
				 enum v4l2_power state)
{
	/* Dummy function, no powerup is needed, as we just use GPIOs
	   (no extra chip) */
	return 0;
}

/**
 * tps61059_ioctl_g_priv - V4L2 sensor interface handler for vidioc_int_g_priv_num
 * @s: pointer to standard V4L2 device structure
 * @p: void pointer to hold flash's private data address
 *
 * Returns device's (flash's) private data area address in p parameter
 */
static int tps61059_ioctl_g_priv(struct v4l2_int_device *s, void *p)
{
	struct tps61059_flash *flash = s->priv;
	return flash->pdata->priv_data_set(p);

}

static struct v4l2_int_ioctl_desc tps61059_ioctl_desc[] = {
	{ vidioc_int_queryctrl_num,
	  (v4l2_int_ioctl_func *)tps61059_ioctl_queryctrl },
	{ vidioc_int_g_ctrl_num,
	  (v4l2_int_ioctl_func *)tps61059_ioctl_g_ctrl },
	{ vidioc_int_s_ctrl_num,
	  (v4l2_int_ioctl_func *)tps61059_ioctl_s_ctrl },
	{ vidioc_int_s_power_num,
	  (v4l2_int_ioctl_func *)tps61059_ioctl_s_power },
	{ vidioc_int_g_priv_num,
	  (v4l2_int_ioctl_func *)tps61059_ioctl_g_priv },
};

static struct v4l2_int_slave tps61059_slave = {
	.ioctls = tps61059_ioctl_desc,
	.num_ioctls = ARRAY_SIZE(tps61059_ioctl_desc),
};

static struct v4l2_int_device tps61059_int_device = {
	.module = THIS_MODULE,
	.name = "TPS61059",
	.type = v4l2_int_type_slave,
	.u = {
		.slave = &tps61059_slave,
	},
};

static int tps61059_probe(struct platform_device *pdev)
{
	struct tps61059_flash *flash;
	struct tps61059_platform_data *pdata = pdev->dev.platform_data;
	int err;

	if (!pdata) {
		dev_err(&pdev->dev, "tps61059 platform data not supplied\n");
		return -ENOENT;
	}

	flash = kzalloc(sizeof(*flash), GFP_KERNEL);
	if (!flash)
		return -ENOMEM;

	/* Don't keep pointer to platform data, copy elements instead */
	flash->pdata = kzalloc(sizeof(*flash->pdata), GFP_KERNEL);
	if (!flash->pdata) {
		err = -ENOMEM;
		goto on_err1;
	}

	flash->pdata->flash_on = pdata->flash_on;
	flash->pdata->flash_off = pdata->flash_off;
	flash->pdata->s_torch_intensity = pdata->s_torch_intensity;
	flash->pdata->priv_data_set = pdata->priv_data_set;

	/* Set flash default values */
	flash->flash_timeout = tps61059_ctrls
			[CTRL_CAMERA_FLASH_TIMEOUT].default_value;
	flash->torch_intensity = tps61059_ctrls
			[CTRL_CAMERA_FLASH_TORCH_INTENSITY].default_value;

	flash->v4l2_int_device = &tps61059_int_device;
	flash->v4l2_int_device->priv = flash;

	flash->dev = &pdev->dev;
	platform_set_drvdata(pdev, flash);

	err = v4l2_int_device_register(flash->v4l2_int_device);
	if (err) {
		dev_err(flash->dev, "Could not register "
				"tps61059 as v4l2_int_device\n");
		goto on_err2;
	}

	return 0;

on_err2:
	platform_set_drvdata(pdev, NULL);
	kfree(flash->pdata);
on_err1:
	kfree(flash);
	return err;
}

static int tps61059_remove(struct platform_device *pdev)
{
	struct tps61059_flash *flash = platform_get_drvdata(pdev);
	int ret = 0;

	ret = del_timer(&flash->strobe_timer);
	if (ret) {
		dev_err(flash->dev, "strobe_timer is still in use!\n");
		return -EBUSY;
	}

	v4l2_int_device_unregister(flash->v4l2_int_device);
	platform_set_drvdata(pdev, NULL);
	kfree(flash->pdata);
	kfree(flash);
	return 0;
}

static struct platform_driver tps61059_driver = {
	.probe	 = tps61059_probe,
	.remove	 = tps61059_remove,
	.driver	 = {
		.name = "tps61059",
	},
};

/**
 * tps61059_init - Module initialisation.
 *
 * Returns 0 if successful, or -ENODEV if device couldn't be initialized, or
 * added as a character device.
 **/
static int __init tps61059_init(void)
{
	return platform_driver_register(&tps61059_driver);
}

/**
 * tps61059_exit - Module cleanup.
 **/
static void __exit tps61059_exit(void)
{
	platform_driver_unregister(&tps61059_driver);
}

late_initcall(tps61059_init);
module_exit(tps61059_exit);

MODULE_AUTHOR("Nikolay Vladimirov <nvladimirov@mm-sol.com>");
MODULE_DESCRIPTION("TPS61059 3430spd GPIO LED flash v4l2 device");
MODULE_LICENSE("GPL");

