/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

static void timer_fn(struct k_timer *timer)
{
	static uint32_t last;
	uint32_t now = k_cycle_get_32();
	uint32_t dt = now - last;
	printk("%s() dt %d\n", __func__, dt);
	last = now;
}

int main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);

	printk("Busy waiting...\n");
	k_busy_wait(300000);

	printk("Entering a syscall...\n");
	__asm__ volatile("syscall");
	printk("came back, all done!\n");

	struct k_timer t;
	k_timer_init(&t, timer_fn, NULL);
	printk("starting timer...\n");
	k_timer_start(&t, K_MSEC(500), K_MSEC(1000));

	while(1);
	return 0;
}
