/*
 * linux/arch/arm/plat-omap/dmtimer.c
 *
 * OMAP Dual-Mode Timers
 *
 * Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
 * Tarun Kanti DebBarma <tarun.kanti@ti.com>
 * Thara Gopinath <thara@ti.com>
 *
 * dmtimer adaptation to platform_driver.
 *
 * Copyright (C) 2005 Nokia Corporation
 * OMAP2 support by Juha Yrjola
 * API improvements and OMAP2 clock framework support by Timo Teras
 *
 * Copyright (C) 2009 Texas Instruments
 * Added OMAP4 support - Santosh Shilimkar <santosh.shilimkar@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the  GNU General Public License along
 * with this program; if not, write  to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/err.h>
#include <linux/platform_device.h>

#include <plat/dmtimer.h>
#include <plat/common.h>
#include <plat/omap-pm.h>

static int dm_timer_count;

#ifdef CONFIG_ARCH_OMAP1
static struct omap_dm_timer omap1_dm_timers[] = {
	{ .phys_base = 0xfffb1400, .irq = INT_1610_GPTIMER1 },
	{ .phys_base = 0xfffb1c00, .irq = INT_1610_GPTIMER2 },
	{ .phys_base = 0xfffb2400, .irq = INT_1610_GPTIMER3 },
	{ .phys_base = 0xfffb2c00, .irq = INT_1610_GPTIMER4 },
	{ .phys_base = 0xfffb3400, .irq = INT_1610_GPTIMER5 },
	{ .phys_base = 0xfffb3c00, .irq = INT_1610_GPTIMER6 },
	{ .phys_base = 0xfffb7400, .irq = INT_1610_GPTIMER7 },
	{ .phys_base = 0xfffbd400, .irq = INT_1610_GPTIMER8 },
};

static LIST_HEAD(omap_timer_list);
static DEFINE_MUTEX(dm_timer_mutex);

/**
 * omap_dm_timer_read_reg - read timer registers in posted and non-posted mode
 * @timer:      timer pointer over which read operation to perform
 * @reg:        lowest byte holds the register offset
 *
 * The posted mode bit is encoded in reg. Note that in posted mode write
 * pending bit must be checked. Otherwise a read of a non completed write
 * will produce an error.
 */
static inline u32 omap_dm_timer_read_reg(struct omap_dm_timer *timer, u32 reg)
{
	int i = 0;

	if (reg >= OMAP_TIMER_WAKEUP_EN_REG)
		reg += timer->func_offset;
	else if (reg >= OMAP_TIMER_STAT_REG)
		reg += timer->intr_offset;

	if (timer->posted) {
		omap_test_timeout(!(readl(timer->io_base +
			((OMAP_TIMER_WRITE_PEND_REG +
			timer->func_offset) & 0xff)) & (reg >> WPSHIFT)),
			MAX_WRITE_PEND_WAIT, i);

		if (WARN_ON_ONCE(i == MAX_WRITE_PEND_WAIT))
			dev_err(&timer->pdev->dev, "read timeout.\n");
	}

	return readl(timer->io_base + (reg & 0xff));
}

/**
 * omap_dm_timer_write_reg - write timer registers in posted and non-posted mode
 * @timer:      timer pointer over which write operation is to perform
 * @reg:        lowest byte holds the register offset
 * @value:      data to write into the register
 *
 * The posted mode bit is encoded in reg. Note that in posted mode the write
 * pending bit must be checked. Otherwise a write on a register which has a
 * pending write will be lost.
 */
static void omap_dm_timer_write_reg(struct omap_dm_timer *timer, u32 reg,
						u32 value)
{
	int i = 0;

	if (reg >= OMAP_TIMER_WAKEUP_EN_REG)
		reg += timer->func_offset;
	else if (reg >= OMAP_TIMER_STAT_REG)
		reg += timer->intr_offset;

	if (timer->posted) {
		omap_test_timeout(!(readl(timer->io_base +
			((OMAP_TIMER_WRITE_PEND_REG +
			timer->func_offset) & 0xff)) & (reg >> WPSHIFT)),
			MAX_WRITE_PEND_WAIT, i);

		if (WARN_ON(i == MAX_WRITE_PEND_WAIT))
			dev_err(&timer->pdev->dev, "write timeout.\n");
	}

	writel(value, timer->io_base + (reg & 0xff));
}

static void omap_timer_save_context(struct omap_dm_timer *timer)
{
	return __omap_dm_timer_read(timer->io_base, reg, timer->posted);
}

static void omap_timer_restore_context(struct omap_dm_timer *timer)
{
	__omap_dm_timer_write(timer->io_base, reg, value, timer->posted);
}

static void omap_dm_timer_wait_for_reset(struct omap_dm_timer *timer)
{
	int c;

	c = 0;
	while (!(omap_dm_timer_read_reg(timer, OMAP_TIMER_SYS_STAT_REG) & 1)) {
		c++;
		if (c > 100000) {
			printk(KERN_ERR "Timer failed to reset\n");
			return;
		}
	}
}

static void omap_dm_timer_reset(struct omap_dm_timer *timer)
{
	int autoidle = 0, wakeup = 0;

	if (!timer->is_early_init)
		__timer_enable(timer);

	if (timer->pdev->id != 1) {
		omap_dm_timer_write_reg(timer, OMAP_TIMER_IF_CTRL_REG, 0x06);
		omap_dm_timer_wait_for_reset(timer);
	}

	/* Enable autoidle on OMAP2+ */
	if (cpu_class_is_omap2())
		autoidle = 1;

	/*
	 * Enable wake-up on OMAP2 CPUs.
	 */
	if (cpu_class_is_omap2())
		wakeup = 1;

	__omap_dm_timer_reset(timer->io_base, autoidle, wakeup);
	timer->posted = 1;
}

void omap_dm_timer_prepare(struct omap_dm_timer *timer)
{
	omap_dm_timer_enable(timer);
	omap_dm_timer_reset(timer);
}

struct omap_dm_timer *omap_dm_timer_request(void)
{
	struct omap_dm_timer *timer = NULL, *t;
	int ret;

	mutex_lock(&dm_timer_mutex);
	list_for_each_entry(t, &omap_timer_list, node) {
		if (t->reserved)
			continue;

		timer = t;
		timer->reserved = 1;
		timer->enabled = 0;
		break;
	}
	mutex_unlock(&dm_timer_mutex);

	if (!timer) {
		pr_debug("%s: free timer not available.\n", __func__);
		return NULL;
	}
	ret = omap_dm_timer_prepare(timer);
	if (ret) {
		timer->reserved = 0;
		return NULL;
	}

	return timer;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_request);

struct omap_dm_timer *omap_dm_timer_request_specific(int id)
{
	struct omap_dm_timer *timer = NULL, *t;
	int ret;

	mutex_lock(&dm_timer_mutex);
	list_for_each_entry(t, &omap_timer_list, node) {
		if (t->pdev->id == id && !t->reserved) {
			timer = t;
			timer->reserved = 1;
			timer->enabled = 0;
			break;
		}
	}
	mutex_unlock(&dm_timer_mutex);

	if (!timer) {
		pr_debug("%s: timer%d not available.\n", __func__, id);
		return NULL;
	}
	ret = omap_dm_timer_prepare(timer);
	if (ret) {
		timer->reserved = 0;
		return NULL;
	}

	return timer;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_request_specific);

int omap_dm_timer_free(struct omap_dm_timer *timer)
{
	unsigned long flags;
	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (!timer->reserved) {
		spin_unlock_irqrestore(&timer->lock, flags);
		return -EINVAL;
	}

	__timer_disable(timer);
	clk_put(timer->fclk);

	timer->reserved = 0;
	timer->context_saved = false;

	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_free);

int omap_dm_timer_enable(struct omap_dm_timer *timer)
{
	unsigned long flags;
	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	__timer_enable(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_enable);

int omap_dm_timer_disable(struct omap_dm_timer *timer)
{
	unsigned long flags;
	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	__timer_disable(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_disable);

int omap_dm_timer_get_irq(struct omap_dm_timer *timer)
{
	if (timer)
		return timer->irq;
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_get_irq);

#if defined(CONFIG_ARCH_OMAP1)

/**
 * omap_dm_timer_modify_idlect_mask - Check if any running timers use ARMXOR
 * @inputmask: current value of idlect mask
 */
__u32 omap_dm_timer_modify_idlect_mask(__u32 inputmask)
{
	int i = 0;
	struct omap_dm_timer *timer = NULL;

	/* If ARMXOR cannot be idled this function call is unnecessary */
	if (!(inputmask & (1 << 1)))
		return inputmask;

	/* If any active timer is using ARMXOR return modified mask */
	mutex_lock(&dm_timer_mutex);
	list_for_each_entry(timer, &omap_timer_list, node) {

		u32 l;

		l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
		if (l & OMAP_TIMER_CTRL_ST) {
			if (((omap_readl(MOD_CONF_CTRL_1) >> (i * 2)) & 0x03) == 0)
				inputmask &= ~(1 << 1);
			else
				inputmask &= ~(1 << 2);
		}
		i++;
	}
	mutex_unlock(&dm_timer_mutex);

	return inputmask;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_modify_idlect_mask);

#else

struct clk *omap_dm_timer_get_fclk(struct omap_dm_timer *timer)
{
	if (timer)
		return timer->fclk;
	return NULL;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_get_fclk);

__u32 omap_dm_timer_modify_idlect_mask(__u32 inputmask)
{
	BUG();

	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_modify_idlect_mask);

#endif

int omap_dm_timer_trigger(struct omap_dm_timer *timer)
{
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (timer->enabled) {
		omap_dm_timer_write_reg(timer, OMAP_TIMER_TRIGGER_REG, 0);
		spin_unlock_irqrestore(&timer->lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&timer->lock, flags);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_trigger);

int omap_dm_timer_start(struct omap_dm_timer *timer)
{
	u32 l;
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (timer->loses_context) {
		__timer_enable(timer);
		if (omap_pm_was_context_lost(&timer->pdev->dev) &&
			timer->context_saved) {
			omap_timer_restore_context(timer);
			timer->context_saved = false;
		}
	}

	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	if (!(l & OMAP_TIMER_CTRL_ST)) {
		l |= OMAP_TIMER_CTRL_ST;
		omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
	}
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_start);

int omap_dm_timer_stop(struct omap_dm_timer *timer)
{
	unsigned long rate = 0;

#ifdef CONFIG_ARCH_OMAP2PLUS
	rate = clk_get_rate(timer->fclk);
#endif

	__omap_dm_timer_stop(timer->io_base, timer->posted, rate);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_stop);

#ifdef CONFIG_ARCH_OMAP1

int omap_dm_timer_set_source(struct omap_dm_timer *timer, int source)
{
	int n = (timer - dm_timers) << 1;
	u32 l;

	l = omap_readl(MOD_CONF_CTRL_1) & ~(0x03 << n);
	l |= source << n;
	omap_writel(l, MOD_CONF_CTRL_1);

	if (timer->loses_context) {
		omap_timer_save_context(timer);
		timer->context_saved = true;
		__timer_disable(timer);
	}
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_stop);

int omap_dm_timer_set_source(struct omap_dm_timer *timer, int source)
{
	if (source < 0 || source >= 3)
		return -EINVAL;

	return __omap_dm_timer_set_source(timer->fclk,
						dm_source_clocks[source]);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_source);

int omap_dm_timer_set_load(struct omap_dm_timer *timer, int autoreload,
			    unsigned int load)
{
	u32 l;

	unsigned long flags;
	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	__timer_enable(timer);
	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	if (autoreload)
		l |= OMAP_TIMER_CTRL_AR;
	else
		l &= ~OMAP_TIMER_CTRL_AR;
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_LOAD_REG, load);

	omap_dm_timer_write_reg(timer, OMAP_TIMER_TRIGGER_REG, 0);
	__timer_disable(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_load);

int omap_dm_timer_set_load_start(struct omap_dm_timer *timer, int autoreload,
                            unsigned int load)
{
	u32 l;
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (timer->loses_context) {
		__timer_enable(timer);
		if (omap_pm_was_context_lost(&timer->pdev->dev) &&
			timer->context_saved) {
			omap_timer_restore_context(timer);
			timer->context_saved = false;
		}
	}

	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	if (autoreload) {
		l |= OMAP_TIMER_CTRL_AR;
		omap_dm_timer_write_reg(timer, OMAP_TIMER_LOAD_REG, load);
	} else {
		l &= ~OMAP_TIMER_CTRL_AR;
	}
	l |= OMAP_TIMER_CTRL_ST;

	__omap_dm_timer_load_start(timer->io_base, l, load, timer->posted);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_load_start);

int omap_dm_timer_set_match(struct omap_dm_timer *timer, int enable,
			     unsigned int match)
{
	u32 l;
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	__timer_enable(timer);
	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	if (enable)
		l |= OMAP_TIMER_CTRL_CE;
	else
		l &= ~OMAP_TIMER_CTRL_CE;
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
	omap_dm_timer_write_reg(timer, OMAP_TIMER_MATCH_REG, match);
	__timer_disable(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_match);

int omap_dm_timer_set_pwm(struct omap_dm_timer *timer, int def_on,
			   int toggle, int trigger)
{
	u32 l;
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	__timer_enable(timer);
	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	l &= ~(OMAP_TIMER_CTRL_GPOCFG | OMAP_TIMER_CTRL_SCPWM |
	       OMAP_TIMER_CTRL_PT | (0x03 << 10));
	if (def_on)
		l |= OMAP_TIMER_CTRL_SCPWM;
	if (toggle)
		l |= OMAP_TIMER_CTRL_PT;
	l |= trigger << 10;
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
	__timer_disable(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_pwm);

int omap_dm_timer_set_prescaler(struct omap_dm_timer *timer, int prescaler)
{
	u32 l;
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	__timer_enable(timer);
	l = omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG);
	l &= ~(OMAP_TIMER_CTRL_PRE | (0x07 << 2));
	if (prescaler >= 0x00 && prescaler <= 0x07) {
		l |= OMAP_TIMER_CTRL_PRE;
		l |= prescaler << 2;
	}
	omap_dm_timer_write_reg(timer, OMAP_TIMER_CTRL_REG, l);
	__timer_disable(timer);
	spin_unlock_irqrestore(&timer->lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_prescaler);

int omap_dm_timer_set_int_enable(struct omap_dm_timer *timer,
				  unsigned int value)
{
	__omap_dm_timer_int_enable(timer->io_base, value);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_set_int_enable);

unsigned int omap_dm_timer_read_status(struct omap_dm_timer *timer)
{
	unsigned long flags;
	unsigned int ret;

	if (WARN_ON(!timer))
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (timer->is_early_init || timer->enabled) {
		ret = omap_dm_timer_read_reg(timer, OMAP_TIMER_STAT_REG);
		spin_unlock_irqrestore(&timer->lock, flags);
		return ret;
	}
	spin_unlock_irqrestore(&timer->lock, flags);
	WARN_ON(!timer->enabled);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_read_status);

int omap_dm_timer_write_status(struct omap_dm_timer *timer, unsigned int value)
{
	__omap_dm_timer_write_status(timer->io_base, value);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_write_status);

unsigned int omap_dm_timer_read_counter(struct omap_dm_timer *timer)
{
	return __omap_dm_timer_read_counter(timer->io_base, timer->posted);
}
EXPORT_SYMBOL_GPL(omap_dm_timer_read_counter);

int omap_dm_timer_write_counter(struct omap_dm_timer *timer, unsigned int value)
{
	unsigned long flags;

	if (!timer)
		return -EINVAL;

	spin_lock_irqsave(&timer->lock, flags);
	if (timer->is_early_init || timer->enabled) {
		omap_dm_timer_write_reg(timer, OMAP_TIMER_COUNTER_REG, value);
		spin_unlock_irqrestore(&timer->lock, flags);
		return 0;
	}
	spin_unlock_irqrestore(&timer->lock, flags);
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(omap_dm_timer_write_counter);

int omap_dm_timers_active(void)
{
	struct omap_dm_timer *timer;

	list_for_each_entry(timer, &omap_timer_list, node) {
		if (!timer->enabled)
			continue;

		if (omap_dm_timer_read_reg(timer, OMAP_TIMER_CTRL_REG) &
		    OMAP_TIMER_CTRL_ST) {
			return 1;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(omap_dm_timers_active);

static int __init omap_dm_timer_init(void)
{
	int ret;
	struct omap_dm_timer *timer;
	struct resource *mem, *irq, *ioarea;
	struct dmtimer_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata) {
		dev_err(&pdev->dev, "%s: no platform data.\n", __func__);
		return -ENODEV;
	}

	spin_lock_init(&dm_timer_lock);

	if (cpu_class_is_omap1()) {
		dm_timers = omap1_dm_timers;
		dm_timer_count = omap1_dm_timer_count;
		map_size = SZ_2K;
	} else if (cpu_is_omap24xx()) {
		dm_timers = omap2_dm_timers;
		dm_timer_count = omap2_dm_timer_count;
		dm_source_names = omap2_dm_source_names;
		dm_source_clocks = omap2_dm_source_clocks;
	} else if (cpu_is_omap34xx()) {
		dm_timers = omap3_dm_timers;
		dm_timer_count = omap3_dm_timer_count;
		dm_source_names = omap3_dm_source_names;
		dm_source_clocks = omap3_dm_source_clocks;
	} else if (cpu_is_omap44xx()) {
		dm_timers = omap4_dm_timers;
		dm_timer_count = omap4_dm_timer_count;
		dm_source_names = omap4_dm_source_names;
		dm_source_clocks = omap4_dm_source_clocks;
	}

	if (cpu_class_is_omap2())
		for (i = 0; dm_source_names[i] != NULL; i++)
			dm_source_clocks[i] = clk_get(NULL, dm_source_names[i]);

	if (cpu_is_omap243x())
		dm_timers[0].phys_base = 0x49018000;

	for (i = 0; i < dm_timer_count; i++) {
		timer = &dm_timers[i];

		/* Static mapping, never released */
		timer->io_base = ioremap(timer->phys_base, map_size);
		BUG_ON(!timer->io_base);

#ifdef CONFIG_ARCH_OMAP2PLUS
		if (cpu_class_is_omap2()) {
			char clk_name[16];
			sprintf(clk_name, "gpt%d_ick", i + 1);
			timer->iclk = clk_get(NULL, clk_name);
			sprintf(clk_name, "gpt%d_fck", i + 1);
			timer->fclk = clk_get(NULL, clk_name);
		}

		/* One or two timers may be set up early for sys_timer */
		if (sys_timer_reserved & (1  << i)) {
			timer->reserved = 1;
			timer->posted = 1;
		}
#endif
	}

	/* add the timer element to the list */
	mutex_lock(&dm_timer_mutex);
	list_add_tail(&timer->node, &omap_timer_list);
	mutex_unlock(&dm_timer_mutex);

	dev_dbg(&pdev->dev, "Device Probed.\n");

	return 0;

err_free_mem:
	kfree(timer);

err_release_ioregion:
	release_mem_region(mem->start, resource_size(mem));

	return ret;
}

/**
 * omap_dm_timer_remove - cleanup a registered timer device
 * @pdev:	pointer to current timer platform device
 *
 * Called by driver framework whenever a timer device is unregistered.
 * In addition to freeing platform resources it also deletes the timer
 * entry from the local list.
 */
static int __devexit omap_dm_timer_remove(struct platform_device *pdev)
{
	struct omap_dm_timer *timer;
	int ret = -EINVAL;

	mutex_lock(&dm_timer_mutex);
	list_for_each_entry(timer, &omap_timer_list, node) {
		if (timer->pdev->id == pdev->id) {
			list_del(&timer->node);
			kfree(timer);
			ret = 0;
			break;
		}
	}
	mutex_unlock(&dm_timer_mutex);
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static struct platform_driver omap_dm_timer_driver = {
	.probe  = omap_dm_timer_probe,
	.remove = omap_dm_timer_remove,
	.driver = {
		.name   = "omap_timer",
	},
};

static int __init omap_dm_timer_driver_init(void)
{
	return platform_driver_register(&omap_dm_timer_driver);
}

static void __exit omap_dm_timer_driver_exit(void)
{
	platform_driver_unregister(&omap_dm_timer_driver);
}

arch_initcall(omap_dm_timer_init);
