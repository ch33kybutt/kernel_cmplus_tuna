/*
 * OMAP Power Management debug routines
 *
 * Copyright (C) 2005 Texas Instruments, Inc.
 * Copyright (C) 2006-2008 Nokia Corporation
 *
 * Written by:
 * Richard Woodruff <r-woodruff2@ti.com>
 * Tony Lindgren
 * Juha Yrjola
 * Amit Kucheria <amit.kucheria@nokia.com>
 * Igor Stoppa <igor.stoppa@nokia.com>
 * Jouni Hogander
 *
 * Based on pm.c for omap2
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <plat/clock.h>
#include <plat/board.h>
#include "powerdomain.h"
#include "clockdomain.h"
#include <plat/dmtimer.h>
#include <plat/omap-pm.h>

#include "cm2xxx_3xxx.h"
#include "prm2xxx_3xxx.h"
#include "pm.h"

u32 enable_off_mode;

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int pm_dbg_init_done;

static int pm_dbg_init(void);

enum {
	DEBUG_FILE_COUNTERS = 0,
	DEBUG_FILE_TIMERS,
	DEBUG_FILE_LAST_COUNTERS,
	DEBUG_FILE_LAST_TIMERS,
};

static const char pwrdm_state_names[][PWRDM_MAX_PWRSTS] = {
	"OFF",
	"RET",
	"INA",
	"ON"
};

void pm_dbg_update_time(struct powerdomain *pwrdm, int prev)
{
	s64 t;

	if (!pm_dbg_init_done)
		return ;

	/* Update timer for previous state */
	t = sched_clock();

	pwrdm->time.state[prev] += t - pwrdm->timer;

	pwrdm->timer = t;
}

static int clkdm_dbg_show_counter(struct clockdomain *clkdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;

	if (strcmp(clkdm->name, "emu_clkdm") == 0 ||
		strcmp(clkdm->name, "wkup_clkdm") == 0 ||
		strncmp(clkdm->name, "dpll", 4) == 0)
		return 0;

	seq_printf(s, "%s->%s (%d)", clkdm->name,
			clkdm->pwrdm.ptr->name,
			atomic_read(&clkdm->usecount));
	seq_printf(s, "\n");

	return 0;
}

static int pwrdm_dbg_show_count_stats(struct powerdomain *pwrdm,
	struct powerdomain_count_stats *stats, struct seq_file *s)
{
	int i;

	seq_printf(s, "%s (%s)", pwrdm->name,
			pwrdm_state_names[pwrdm->state]);

	for (i = 0; i < PWRDM_MAX_PWRSTS; i++)
		seq_printf(s, ",%s:%d", pwrdm_state_names[i],
			stats->state[i]);

	seq_printf(s, ",RET-LOGIC-OFF:%d", stats->ret_logic_off);
	for (i = 0; i < pwrdm->banks; i++)
		seq_printf(s, ",RET-MEMBANK%d-OFF:%d", i + 1,
				stats->ret_mem_off[i]);

	seq_printf(s, "\n");

	return 0;
}

static int pwrdm_dbg_show_time_stats(struct powerdomain *pwrdm,
	struct powerdomain_time_stats *stats, struct seq_file *s)
{
	int i;
	u64 total = 0;

	seq_printf(s, "%s (%s)", pwrdm->name,
		pwrdm_state_names[pwrdm->state]);

	for (i = 0; i < 4; i++)
		total += stats->state[i];

	for (i = 0; i < 4; i++)
		seq_printf(s, ",%s:%lld (%lld%%)", pwrdm_state_names[i],
			stats->state[i],
			total ? div64_u64(stats->state[i] * 100, total) : 0);

	seq_printf(s, "\n");

	return 0;
}

static int pwrdm_dbg_show_counter(struct powerdomain *pwrdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;

	if (strcmp(pwrdm->name, "emu_pwrdm") == 0 ||
		strcmp(pwrdm->name, "wkup_pwrdm") == 0 ||
		strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	if (pwrdm->state != pwrdm_read_pwrst(pwrdm))
		printk(KERN_ERR "pwrdm state mismatch(%s) %d != %d\n",
			pwrdm->name, pwrdm->state, pwrdm_read_pwrst(pwrdm));

	pwrdm_dbg_show_count_stats(pwrdm, &pwrdm->count, s);

	return 0;
}

static int pwrdm_dbg_show_timer(struct powerdomain *pwrdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;

	if (strcmp(pwrdm->name, "emu_pwrdm") == 0 ||
		strcmp(pwrdm->name, "wkup_pwrdm") == 0 ||
		strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	pwrdm_state_switch(pwrdm);

	pwrdm_dbg_show_time_stats(pwrdm, &pwrdm->time, s);

	return 0;
}

static int pwrdm_dbg_show_last_counter(struct powerdomain *pwrdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;
	struct powerdomain_count_stats stats;
	int i;

	if (strcmp(pwrdm->name, "emu_pwrdm") == 0 ||
		strcmp(pwrdm->name, "wkup_pwrdm") == 0 ||
		strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	stats = pwrdm->count;
	for (i = 0; i < PWRDM_MAX_PWRSTS; i++)
		stats.state[i] -= pwrdm->last_count.state[i];
	for (i = 0; i < PWRDM_MAX_MEM_BANKS; i++)
		stats.ret_mem_off[i] -= pwrdm->last_count.ret_mem_off[i];
	stats.ret_logic_off -= pwrdm->last_count.ret_logic_off;

	pwrdm->last_count = pwrdm->count;

	pwrdm_dbg_show_count_stats(pwrdm, &stats, s);

	return 0;
}

static int pwrdm_dbg_show_last_timer(struct powerdomain *pwrdm, void *user)
{
	struct seq_file *s = (struct seq_file *)user;
	struct powerdomain_time_stats stats;
	int i;

	stats = pwrdm->time;
	for (i = 0; i < PWRDM_MAX_PWRSTS; i++)
		stats.state[i] -= pwrdm->last_time.state[i];

	pwrdm->last_time = pwrdm->time;

	pwrdm_dbg_show_time_stats(pwrdm, &stats, s);

	return 0;
}

static int pm_dbg_show_counters(struct seq_file *s, void *unused)
{
	pwrdm_for_each(pwrdm_dbg_show_counter, s);
	if (cpu_is_omap44xx())
		seq_printf(s, "DEVICE-OFF:%d\n", omap4_device_off_counter);
	clkdm_for_each(clkdm_dbg_show_counter, s);

	return 0;
}

static int pm_dbg_show_timers(struct seq_file *s, void *unused)
{
	pwrdm_for_each(pwrdm_dbg_show_timer, s);
	return 0;
}

static int pm_dbg_show_last_counters(struct seq_file *s, void *unused)
{
	pwrdm_for_each(pwrdm_dbg_show_last_counter, s);
	return 0;
}

static int pm_dbg_show_last_timers(struct seq_file *s, void *unused)
{
	pwrdm_for_each(pwrdm_dbg_show_last_timer, s);
	return 0;
}

static int pm_dbg_open(struct inode *inode, struct file *file)
{
	switch ((int)inode->i_private) {
	case DEBUG_FILE_COUNTERS:
		return single_open(file, pm_dbg_show_counters,
			&inode->i_private);
	case DEBUG_FILE_TIMERS:
		return single_open(file, pm_dbg_show_timers,
			&inode->i_private);
	case DEBUG_FILE_LAST_COUNTERS:
		return single_open(file, pm_dbg_show_last_counters,
			&inode->i_private);
	case DEBUG_FILE_LAST_TIMERS:
	default:
		return single_open(file, pm_dbg_show_last_timers,
			&inode->i_private);
	};
}

static const struct file_operations debug_fops = {
	.open           = pm_dbg_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int pwrdm_suspend_get(void *data, u64 *val)
{
	int ret = -EINVAL;

	if (cpu_is_omap34xx())
		ret = omap3_pm_get_suspend_state((struct powerdomain *)data);
	*val = ret;

	if (ret >= 0)
		return 0;
	return *val;
}

static int pwrdm_suspend_set(void *data, u64 val)
{
	if (cpu_is_omap34xx())
		return omap3_pm_set_suspend_state(
			(struct powerdomain *)data, (int)val);
	return -EINVAL;
}

DEFINE_SIMPLE_ATTRIBUTE(pwrdm_suspend_fops, pwrdm_suspend_get,
			pwrdm_suspend_set, "%llu\n");

#ifdef CONFIG_PM_ADVANCED_DEBUG
static bool is_addr_valid()
{
	int saved_reg_addr_max = 0;
	/* Only for OMAP4 for the timebeing */
	if (!cpu_is_omap44xx())
		return false;

	saved_reg_num = (saved_reg_num > PM_DEBUG_MAX_SAVED_REGS) ?
		PM_DEBUG_MAX_SAVED_REGS : saved_reg_num;

	saved_reg_addr_max = saved_reg_addr + (saved_reg_num * 4) - 1;

	if (saved_reg_addr >= PM_DEBUG_PRM_MIN &&
		saved_reg_addr_max <= PM_DEBUG_PRM_MAX)
			return true;
	if (saved_reg_addr >= PM_DEBUG_CM1_MIN &&
		saved_reg_addr_max <= PM_DEBUG_CM1_MAX)
			return true;
	if (saved_reg_addr >= PM_DEBUG_CM2_MIN &&
		saved_reg_addr_max <= PM_DEBUG_CM2_MAX)
			return true;
	return false;
}

void omap4_pm_suspend_save_regs()
{
	int i = 0;
	if (!saved_reg_num || !is_addr_valid())
		return;

	saved_reg_num_used = saved_reg_num;

	for (i = 0; i < saved_reg_num; i++) {
		saved_reg_buff[1][i] = omap_readl(saved_reg_addr + (i*4));
		saved_reg_buff[0][i] = saved_reg_addr + (i*4);
	}
	return;
}
#endif

static int __init pwrdms_setup(struct powerdomain *pwrdm, void *dir)
{
	int i;
	s64 t;
	struct dentry *d;

	t = sched_clock();

	for (i = 0; i < 4; i++)
		pwrdm->time.state[i] = 0;

	pwrdm->timer = t;

	if (strncmp(pwrdm->name, "dpll", 4) == 0)
		return 0;

	d = debugfs_create_dir(pwrdm->name, (struct dentry *)dir);

	(void) debugfs_create_file("suspend", S_IRUGO|S_IWUSR, d,
			(void *)pwrdm, &pwrdm_suspend_fops);

	return 0;
}

static int option_get(void *data, u64 *val)
{
	u32 *option = data;

	if (option == &enable_off_mode) {
		enable_off_mode = off_mode_enabled;
	}

	*val = *option;
#ifdef CONFIG_PM_ADVANCED_DEBUG
	if (option == &saved_reg_addr) {
		int i;
		for (i = 0; i < saved_reg_num_used; i++)
			pr_info(" %x = %x\n", saved_reg_buff[0][i],
				saved_reg_buff[1][i]);
	}
#endif

	return 0;
}

static int option_set(void *data, u64 val)
{
	u32 *option = data;

	*option = val;

	if (option == &enable_off_mode) {
		if (val)
			omap_pm_enable_off_mode();
		else
			omap_pm_disable_off_mode();
		if (cpu_is_omap34xx())
			omap3_pm_off_mode_enable(val);
	}

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(pm_dbg_option_fops, option_get, option_set, "%llu\n");

static int __init pm_dbg_init(void)
{
	struct dentry *d;

	if (pm_dbg_init_done)
		return 0;

	d = debugfs_create_dir("pm_debug", NULL);
	if (IS_ERR(d))
		return PTR_ERR(d);

	(void) debugfs_create_file("count", S_IRUGO,
		d, (void *)DEBUG_FILE_COUNTERS, &debug_fops);
	(void) debugfs_create_file("time", S_IRUGO,
		d, (void *)DEBUG_FILE_TIMERS, &debug_fops);
	(void) debugfs_create_file("last_count", S_IRUGO,
		d, (void *)DEBUG_FILE_LAST_COUNTERS, &debug_fops);
	(void) debugfs_create_file("last_time", S_IRUGO,
		d, (void *)DEBUG_FILE_LAST_TIMERS, &debug_fops);

	pwrdm_for_each(pwrdms_setup, (void *)d);

	(void) debugfs_create_file("enable_off_mode", S_IRUGO | S_IWUSR, d,
				   &enable_off_mode, &pm_dbg_option_fops);
	pm_dbg_init_done = 1;

	return 0;
}
arch_initcall(pm_dbg_init);

#endif
