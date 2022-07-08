/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/zephyr.h>

K_SEM_DEFINE(sem, 0, 0);
K_MUTEX_DEFINE(mut);
K_CONDVAR_DEFINE(cv);

void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);

	k_sem_give(&sem);
	k_sem_take(&sem, K_FOREVER);

	k_mutex_lock(&mut, K_FOREVER);
	k_condvar_wait(&cv, &mut, K_NO_WAIT);
	k_mutex_unlock(&mut);

	k_condvar_signal(&cv);
}
