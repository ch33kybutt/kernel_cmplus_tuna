/* arch/arm/mach-omap2/board-tuna-vibrator.c
 *
 * Copyright (C) 2011 Samsung Electronics Co. Ltd. All Rights Reserved.
 * Author: Rom Lemarchand <rlemarchand@sta.samsung.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/hrtimer.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/mutex.h>
#include <asm/mach-types.h>
#include <plat/dmtimer.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>

#include <../../../drivers/staging/android/timed_output.h>

#include "mux.h"
#include "board-tuna.h"

struct vibe {
    struct attribute attr;
    int max_pwmduty;
};

static struct vibe vibeattr = {
    .attr.name="pwmduty",
    .attr.mode = 0644,
    .max_pwmduty = 1450,
};

static struct attribute * myattr[] = {
    &vibeattr.attr,
    NULL
};


/* Vibrator enable pin is changed on Rev 05 to block not intended vibration. */
#define GPIO_MOTOR_EN		162
#define GPIO_MOTOR_EN_REV05	54

#define VIB_GPTIMER_NUM		10
#define PWM_DUTY_MAX		1450
#define MAX_TIMEOUT		10000 /* 10s */

static struct vibrator {
	struct wake_lock wklock;
	struct hrtimer timer;
	struct mutex lock;
	struct omap_dm_timer *gptimer;
	bool enabled;
	unsigned gpio_en;
} vibdata;

static void vibrator_off(void)
{
	if (!vibdata.enabled)
		return;
	omap_dm_timer_stop(vibdata.gptimer);
	gpio_set_value(vibdata.gpio_en, 0);
	vibdata.enabled = false;
	wake_unlock(&vibdata.wklock);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibdata.timer)) {
		ktime_t r = hrtimer_get_remaining(&vibdata.timer);
		return ktime_to_ms(r);
	}

	return 0;
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	mutex_lock(&vibdata.lock);

	/* cancel previous timer and set GPIO according to value */
	hrtimer_cancel(&vibdata.timer);

	if (value) {
		wake_lock(&vibdata.wklock);

		gpio_set_value(vibdata.gpio_en, 1);
		omap_dm_timer_start(vibdata.gptimer);

		vibdata.enabled = true;

		if (value > 0) {
			if (value > MAX_TIMEOUT)
				value = MAX_TIMEOUT;

			hrtimer_start(&vibdata.timer,
				ns_to_ktime((u64)value * NSEC_PER_MSEC),
				HRTIMER_MODE_REL);
		}
	} else {
		vibrator_off();
	}

	mutex_unlock(&vibdata.lock);
}

static struct timed_output_dev to_dev = {
	.name		= "vibrator",
	.get_time	= vibrator_get_time,
	.enable		= vibrator_enable,
};

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	vibrator_off();
	return HRTIMER_NORESTART;
}

void init_pwm_values(void) 
{
        omap_dm_timer_set_load(vibdata.gptimer, 1, -vibeattr.max_pwmduty);
        omap_dm_timer_set_match(vibdata.gptimer, 1, -vibeattr.max_pwmduty+10);
        omap_dm_timer_set_pwm(vibdata.gptimer, 0, 1,
                OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
        omap_dm_timer_enable(vibdata.gptimer);
        omap_dm_timer_write_counter(vibdata.gptimer, -2);
        omap_dm_timer_disable(vibdata.gptimer);
}

static int __init vibrator_init(void)
{
	int ret;

	vibdata.enabled = false;

	hrtimer_init(&vibdata.timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibdata.timer.function = vibrator_timer_func;

	vibdata.gptimer = omap_dm_timer_request_specific(VIB_GPTIMER_NUM);
	if (vibdata.gptimer == NULL)
		return -1;

	ret = omap_dm_timer_set_source(vibdata.gptimer,
		OMAP_TIMER_SRC_SYS_CLK);
	if (ret < 0)
		goto err_dm_timer_src;

/*	omap_dm_timer_set_load(vibdata.gptimer, 1, -vibeattr.max_pwmduty);
	omap_dm_timer_set_match(vibdata.gptimer, 1, -vibeattr.max_pwmduty+10);
	omap_dm_timer_set_pwm(vibdata.gptimer, 0, 1,
		OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);
	omap_dm_timer_enable(vibdata.gptimer);
	omap_dm_timer_write_counter(vibdata.gptimer, -2);
	omap_dm_timer_disable(vibdata.gptimer);*/

	init_pwm_values();

	wake_lock_init(&vibdata.wklock, WAKE_LOCK_SUSPEND, "vibrator");
	mutex_init(&vibdata.lock);

	ret = timed_output_dev_register(&to_dev);
	if (ret < 0)
		goto err_to_dev_reg;

	return 0;

err_to_dev_reg:
	mutex_destroy(&vibdata.lock);
	wake_lock_destroy(&vibdata.wklock);

err_dm_timer_src:
	omap_dm_timer_free(vibdata.gptimer);
	vibdata.gptimer = NULL;

	return -1;
}

static int __init omap4_tuna_vibrator_init(void)
{
	int ret;

	if (!machine_is_tuna())
		return 0;

	vibdata.gpio_en = (omap4_tuna_get_revision() >= 5) ?
			GPIO_MOTOR_EN_REV05 : GPIO_MOTOR_EN;

	omap_mux_init_gpio(vibdata.gpio_en, OMAP_PIN_OUTPUT |
						OMAP_PIN_OFF_OUTPUT_LOW);
	omap_mux_init_signal("dpm_emu18.dmtimer10_pwm_evt", OMAP_PIN_OUTPUT);

	ret = gpio_request(vibdata.gpio_en, "vibrator-en");
	if (ret)
		return ret;

	gpio_direction_output(vibdata.gpio_en, 0);

	ret = vibrator_init();
	if (ret < 0)
		gpio_free(vibdata.gpio_en);

	return ret;
}

/*
 * This is needed because the vibrator is dependent on omap_dm_timers which get
 * initialized at device_init time
 */
late_initcall(omap4_tuna_vibrator_init);

static ssize_t default_show(struct kobject *kobj, struct attribute *attr,
        char *buf)
{
    struct vibe *v = container_of(attr, struct vibe, attr);
    return scnprintf(buf, PAGE_SIZE, "%d\n", v->max_pwmduty);
}

static ssize_t default_store(struct kobject *kobj, struct attribute *attr,
        const char *buf, size_t len)
{
    struct vibe *v = container_of(attr, struct vibe, attr);
    sscanf(buf, "%d", &v->max_pwmduty);
    if (v->max_pwmduty > 1450)
      v->max_pwmduty = 1450;
    init_pwm_values();
    return sizeof(int);
}

static struct sysfs_ops myops = {
    .show = default_show,
    .store = default_store,
};

static struct kobj_type mytype = {
    .sysfs_ops = &myops,
    .default_attrs = myattr,
};

struct kobject *mykobj;
static int __init vibe_module_init(void)
{
    int err = -1;
    mykobj = kzalloc(sizeof(*mykobj), GFP_KERNEL);
    if (mykobj) {
        kobject_init(mykobj, &mytype);
        if (kobject_add(mykobj, NULL, "%s", "vibe")) {
             err = -1;
             printk("Sysfs creation failed\n");
             kobject_put(mykobj);
             mykobj = NULL;
        }
        err = 0;
    }
    return err;
}

static void __exit vibe_module_exit(void)
{
    if (mykobj) {
        kobject_put(mykobj);
        kfree(mykobj);
    }
}

module_init(vibe_module_init);
module_exit(vibe_module_exit);
MODULE_LICENSE("GPL");

