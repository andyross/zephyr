/* Copyright (c) 2022 Google LLC.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/zephyr.h>

#define ARRAYSZ 12

/* Do bad stuff through a function interface to dodge the compiler
 * warnings and optimizer interactions
 */
int read_elem(int *array, int idx);
void write_elem(int *array, int idx, int val);
void set_bad_ptr(void);
extern int global_array[];
extern int *bad_ptr;

/* Default ASAN behavior will halt after the first error and not
 * execute the entire sample.  Build with -fsanitizer-recover=all and
 * set ASAN_OPTIONS=halt_on_error=0 in the environment to see all the
 * failures.
 */
void main(void)
{
	int local_array[12];

	printk("Hello World! %s\n", CONFIG_BOARD);

	printk("Reading item below global array:\n");
	printk("  got %d\n", read_elem(global_array, -1));

	printk("Reading item above global array:\n");
	printk("  got %d\n", read_elem(global_array, ARRAYSZ));

	printk("Reading item below local array:\n");
	printk("  got %d\n", read_elem(local_array, -1));

	printk("Reading item above local array:\n");
	printk("  got %d\n", read_elem(local_array, ARRAYSZ));

	printk("Writing item below global array:\n");
	write_elem(global_array, -1, 0);

	printk("Writing item above global array:\n");
	write_elem(global_array, ARRAYSZ, 0);

	printk("Writing item below local array:\n");
	write_elem(local_array, -1, 0);

	printk("Writing item above local array:\n");
	write_elem(local_array, ARRAYSZ, 0);

	/* Note: these two checks against stack memory of our
	 * since-returned callee don't fail on x86_64, only on 32 bit
	 * builds.  ASAN isn't able to disambiguate the red zone
	 */
	set_bad_ptr();
	printk("Reading stack memory after return (%p):\n", bad_ptr);
	printk("  got %d\n", *bad_ptr);

	printk("Writing stack memory after return:\n");
	*bad_ptr = 0;

	printk("Address sanitizer test complete\n");
}
