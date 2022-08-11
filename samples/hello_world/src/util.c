/* Copyright (c) 2022 Google LLC.
 * SPDX-License-Identifier: Apache-2.0
 */

/* These live in their own file to prevent the optimizer from "fixing" them */

#define ARRAYSZ 12

int global_array[ARRAYSZ];

int *bad_ptr;

int read_elem(int *array, int idx)
{
	return array[idx];
}

void write_elem(int *array, int idx, int val)
{
	array[idx] = val;
}

void set_bad_ptr(void)
{
	int val[3]; /* extra elements so writes don't actually smash the stack! */
	bad_ptr = &val[1];
}
