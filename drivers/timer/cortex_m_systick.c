/*
 * Copyright (c) 2018 Intel Corporation
 * Copyright (c) 2021 Alexander Wachter
 * 
 * SPDX-License-Identifier: Apache-2.0
 */
#include <drivers/timer/system_timer.h>
#include <sys_clock.h>
#include <spinlock.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>

#define COUNTER_MAX 0x00ffffff
#define TIMER_STOPPED 0xff000000

#define CYC_PER_TICK (sys_clock_hw_cycles_per_sec()	\
		      / CONFIG_SYS_CLOCK_TICKS_PER_SEC)
#define MAX_TICKS ((COUNTER_MAX / CYC_PER_TICK) - 1)
#define MAX_CYCLES (MAX_TICKS * CYC_PER_TICK)

/* Minimum cycles in the future to try to program.  Note that this is
 * NOT simply "enough cycles to get the counter read and reprogrammed
 * reliably" -- it becomes the minimum value of the LOAD register, and
 * thus reflects how much time we can reliably see expire between
 * calls to elapsed() to read the COUNTFLAG bit.  So it needs to be
 * set to be larger than the maximum time the interrupt might be
 * masked.  Choosing a fraction of a tick is probably a good enough
 * default, with an absolute minimum of 1k cyc.
 */
#define MIN_DELAY MAX(1024, (CYC_PER_TICK/16))

#define TICKLESS (IS_ENABLED(CONFIG_TICKLESS_KERNEL))

static struct k_spinlock lock;

uint32_t cycle_count;
static uint32_t unannounced;

static uint32_t elapsed(void)
{
	uint32_t val1 = SysTick->VAL;	/* A */
	uint32_t ctrl = SysTick->CTRL;	/* B */
	uint32_t val2 = SysTick->VAL;	/* C */

	/* SysTick behavior: The counter wraps at zero automatically,
	 * setting the COUNTFLAG field of the CTRL register when it
	 * does.  Reading the control register automatically clears
	 * that field.
	 *
	 * If the count wrapped...
	 * 1) Before A then COUNTFLAG will be set and val1 >= val2
	 * 2) Between A and B then COUNTFLAG will be set and val1 < val2
	 * 3) Between B and C then COUNTFLAG will be clear and val1 < val2
	 * 4) After C we'll see it next time
	 *
	 * So the count in val2 is post-wrap and last_load needs to be
	 * added if and only if COUNTFLAG is set or val1 < val2.
	 */
	if ((ctrl & SysTick_CTRL_COUNTFLAG_Msk)
	    || (val1 < val2)) {
		unannounced += SysTick->LOAD;

		/* We know there was a wrap, but we might not have
		 * seen it in CTRL, so clear it. */
		(void)SysTick->CTRL;
	}

	return (SysTick->LOAD - val2) + unannounced;
}

static inline void announce_tickless(uint32_t announce_cyc)
{
	const uint32_t tobe_announced = announce_cyc / CYC_PER_TICK;
	unannounced = announce_cyc - tobe_announced * CYC_PER_TICK;
	sys_clock_announce(tobe_announced);
}

/* Callout out of platform assembly, not hooked via IRQ_CONNECT... */
void sys_clock_isr(void *arg)
{
	ARG_UNUSED(arg);

	cycle_count += SysTick->LOAD;

	if (TICKLESS) {
		announce_tickless(elapsed());
	} else {
		sys_clock_announce(1);
	}

	z_arm_int_exit();
}

int sys_clock_driver_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	NVIC_SetPriority(SysTick_IRQn, _IRQ_PRIO_OFFSET);

	SysTick->LOAD = CYC_PER_TICK - 1;
	SysTick->VAL = 0; /* resets timer to LOAD */
	SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk |
			  SysTick_CTRL_TICKINT_Msk |
			  SysTick_CTRL_CLKSOURCE_Msk);
	return 0;
}

static inline void set_next_timeout(uint32_t ticks)
{
	uint32_t next_to_cycles = MAX(ticks * CYC_PER_TICK - elapsed(), MIN_DELAY);

	unannounced = elapsed();
	SysTick->LOAD = next_to_cycles;
	SysTick->VAL = 0;
}

/**
 * @brief shorten the next timeout to elapse at ticks
 * 
 * This function sets the next announcement to ticks.
 * The next announcement is always shorter then the one actually set.
 */
void sys_clock_set_timeout(int32_t ticks, bool idle)
{
	/* Fast CPUs and a 24 bit counter mean that even idle systems
	 * need to wake up multiple times per second.  If the kernel
	 * allows us to miss tick announcements in idle, then shut off
	 * the counter. (Note: we can assume if idle==true that
	 * interrupts are already disabled)
	 */
	
	if (IS_ENABLED(CONFIG_TICKLESS_KERNEL) && idle && ticks == K_TICKS_FOREVER) {
		SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
		return;
	}

	if (!TICKLESS) {
		return;
	}

	ticks = (ticks == K_TICKS_FOREVER) ? MAX_TICKS : ticks;
	ticks = CLAMP(ticks - 1, 0, (int32_t)MAX_TICKS);

	k_spinlock_key_t key = k_spin_lock(&lock);

	set_next_timeout(ticks);

	k_spin_unlock(&lock, key);
}

uint32_t sys_clock_elapsed(void)
{
	if (!TICKLESS) {
		return 0;
	}

	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t cyc = elapsed();
	k_spin_unlock(&lock, key);
	return cyc / CYC_PER_TICK;
}

uint32_t sys_clock_cycle_get_32(void)
{
	k_spinlock_key_t key = k_spin_lock(&lock);
	uint32_t ret = elapsed() + cycle_count;

	k_spin_unlock(&lock, key);
	return ret;
}

void sys_clock_idle_exit(void)
{
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void sys_clock_disable(void)
{
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}
