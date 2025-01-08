/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

/* Dirty trick to unit test the interrupt exit paths */
#define z_get_next_switch_handle(p) next_sh

#include "arm-m-switch.c"

// Stuff to test:
// 0. Take an interrupt and return alive
// 1. Cross product of:
//    + Contexts saved from {init, switch, switch+fpu, irq, irq+fpu}
//    + Restore via {switch, irq}
// 2. Nested interrupts shouldn't do anything funny (hard to do here though)z

char stack[4096];

void *main_sh, *my_sh;

/* Aforementioned dirty trick: this value becomes the "next swtich
 * handle" examined by the interrupt exit code in lieu of whatever the
 * scheduler would return.
 */
void *next_sh;

int sum;
float fsum;

void my_fn(void *a, void *b, void *c)
{
	printk("%s:%d\n", __func__, __LINE__);

	__ASSERT_NO_MSG((int)a == 0);
	__ASSERT_NO_MSG((int)b == 1);
	__ASSERT_NO_MSG((int)c == 2);

	fsum += 0.5f;

	arm_m_switch(main_sh, &my_sh);
}

void my_svc(void)
{
	printk("%s:%d\n", __func__, __LINE__);

	arm_m_exc_tail();
}

int main(void)
{
	/* Hijack the vector table by copying it into writable RAM */
	static uint32_t __aligned(1024) vectors[256];
	uint32_t *vtor_p = (void *)0xe000ed08;
	uint32_t *vtor = (void *)*vtor_p;
	extern char _vector_start, _vector_end;
	int nv = (&_vector_end - &_vector_start) / sizeof(uint32_t);

	printk("vtor @%p\n", vtor);
	for (int i = 0; i < nv; i++) {
		vectors[i] = vtor[i];
	}
	*vtor_p = (uint32_t) &vectors[0];
	vtor = (void *)*vtor_p;
	printk("vtor now @%p\n", vtor);

	/* And hook the SVC call with our own function above, allowing
	 * us direct access to interrupt entry
	 */
	vtor[11] = (int)my_svc;
	printk("vtor[11] == %p (my_svc == %p)\n", (void*)vtor[11], my_svc);

	/* "register" variables don't strictly force the compiler not
	 * to spill them, but inspecting the generated code shows it's
	 * leaving them in place across the switch.
	 */
	register int A = 1;
	register int B = 2;
	register int C = 3;
	register int D = 4;
	register int E = 5;

	register float F = 6.0f;

	fsum += F;
	sum += A + B + C + D + E;

	/* Hit an interrupt and make sure CPU state doesn't get messed up */
	printk("Invoking SVC\n");
	__asm__ volatile("svc 0");

	__ASSERT_NO_MSG(A == 1);
	__ASSERT_NO_MSG(B == 2);
	__ASSERT_NO_MSG(C == 3);
	__ASSERT_NO_MSG(D == 4);
	__ASSERT_NO_MSG(E == 5);
	__ASSERT_NO_MSG(F == 6);

	/* Now likewise switch to and from a foreign stack and check */
	my_sh = arm_m_new_stack(stack, sizeof(stack), my_fn, (void*)0, (void*)1, (void*)2);

	printk("Switching to initialized handle...\n");
	arm_m_switch(my_sh, &main_sh);
	printk("...and back\n");

	__ASSERT_NO_MSG(A == 1);
	__ASSERT_NO_MSG(B == 2);
	__ASSERT_NO_MSG(C == 3);
	__ASSERT_NO_MSG(D == 4);
	__ASSERT_NO_MSG(E == 5);
	__ASSERT_NO_MSG(F == 6);

	/* Do it again, except via interrupt this time */

	fsum -= F;
	sum -= A + B + C + D + E;

	printk("DONE!\n");
	return 0;
}
