/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include "arm-m-switch.c"

char stack[4096];

void *main_sh, *my_sh;

void my_fn(void *a, void *b, void *c)
{
	printk("%s:%d\n", __func__, __LINE__);

	__ASSERT_NO_MSG((int)a == 0);
	__ASSERT_NO_MSG((int)b == 1);
	__ASSERT_NO_MSG((int)c == 2);

	arm_m_switch(main_sh, &my_sh);
}

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	printk("%s:%d\n", __func__, __LINE__);
	my_sh = arm_m_new_stack(stack, sizeof(stack), my_fn, (void*)0, (void*)1, (void*)2);

	printk("%s:%d\n", __func__, __LINE__);
	arm_m_switch(my_sh, &main_sh);
	printk("%s:%d\n", __func__, __LINE__);

	return 0;
}
