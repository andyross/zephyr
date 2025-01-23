#include <zephyr/kernel.h>
#include <zephyr/linker/linker-defs.h>
#include <zephyr/ztest.h>
#include <kernel_arch_func.h>

char stack[4096];

void *main_sh, *my_sh;

/* Aforementioned dirty trick: this value becomes the "next swtich
 * handle" examined by the interrupt exit code in lieu of whatever the
 * scheduler would return.
 */
void *next_sh;

int sum;

extern void *arm_m_last_switch_handle;

void *z_get_next_switch_handle(void *interrupted)
{
	return next_sh;
}


void my_fn(void *a, void *b, void *c)
{
	printk("%s:%d\n", __func__, __LINE__);

	void *psplim;
	__asm__ volatile("mrs %0, psplim" : "=r"(psplim));
	printk("In my_fn, PSPLIM = %p\n", psplim);

	__ASSERT_NO_MSG((int)a == 0);
	__ASSERT_NO_MSG((int)b == 1);
	__ASSERT_NO_MSG((int)c == 2);

	register int A = 11;
	register int B = 12;
	register int C = 13;
	register int D = 14;
	register int E = 15;

	for (int n = 0; /**/; n++) {
		printk("%s:%d iter %d\n", __func__, __LINE__, n);

		if (arm_m_last_switch_handle) {
			printk("Using exception handle @ %p\n", arm_m_last_switch_handle);
			main_sh = arm_m_last_switch_handle;
			arm_m_last_switch_handle = NULL;
		}

		arm_m_switch(main_sh, &my_sh);

		__ASSERT_NO_MSG(A == 11);
		__ASSERT_NO_MSG(B == 12);
		__ASSERT_NO_MSG(C == 13);
		__ASSERT_NO_MSG(D == 14);
		__ASSERT_NO_MSG(E == 15);
	}
}

void my_svc(void)
{
	printk("%s:%d\n", __func__, __LINE__);

	void *psp;
	__asm__ volatile("mrs %0, psp" : "=r"(psp));

	arm_m_exc_tail();
}

//int main(void)
ZTEST(arm_m_switch, smoke)
{
	void *psplim;
	__asm__ volatile("mrs %0, psplim" : "=r"(psplim));
	printk("In main, PSPLIM = %p\n", psplim);

	/* "register" variables don't strictly force the compiler not
	 * to spill them, but inspecting the generated code shows it's
	 * leaving them in place across the switch.
	 */
	register int A = 1;
	register int B = 2;
	register int C = 3;
	register int D = 4;
	register int E = 5;

#ifdef CONFIG_FPU_SHARING
	/* Prime all the FPU registers with something I can recognize in a debugger */
	uint32_t sregs[32];
	for(int i = 0; i < 32; i++) {
		sregs[i] = 0x3f800000 + i;
	}
	__asm__ volatile("vldm %0, {s0-s31}" :: "r"(sregs));
#endif

	sum += A + B + C + D + E;

	/* Hit an interrupt and make sure CPU state doesn't get messed up */
	printk("Invoking SVC\n");
	__asm__ volatile("svc 0");
	printk("...back\n");

	__ASSERT_NO_MSG(A == 1);
	__ASSERT_NO_MSG(B == 2);
	__ASSERT_NO_MSG(C == 3);
	__ASSERT_NO_MSG(D == 4);
	__ASSERT_NO_MSG(E == 5);

	/* Now likewise switch to and from a foreign stack and check */
	my_sh = arm_m_new_stack(stack, sizeof(stack), my_fn, (void*)0, (void*)1, (void*)2);

	int cycles = 16;
	for(int n = 0; n < cycles; n++) {
		printk("main() switching to my_fn() (iter %d)...\n", n);
		arm_m_switch(my_sh, &main_sh);
		printk("...and back\n");

		__ASSERT_NO_MSG(A == 1);
		__ASSERT_NO_MSG(B == 2);
		__ASSERT_NO_MSG(C == 3);
		__ASSERT_NO_MSG(D == 4);
		__ASSERT_NO_MSG(E == 5);
	}

	/* Do it again, except via interrupt this time */
	printk("Switch via interrupt...\n");
	next_sh = my_sh;
	__asm__ volatile("svc 0");
	printk("back\n");

	sum -= A + B + C + D + E;
}

/* Makes a copy of the vector table in writable RAM (it's generally in
 * a ROM section), redirects it, and hooks the SVC interrupt with our
 * own code above so we can catch direct interrupts.
 */
void *vector_hijack(void)
{
	static uint32_t __aligned(1024) vectors[256];
	uint32_t *vtor_p = (void *)0xe000ed08;
	uint32_t *vtor = (void *)*vtor_p;

	/* Vector count: _vector_start/end set by the linker. */
	int nv = (&_vector_end[0] - &_vector_start[0]) / sizeof(uint32_t);

	printk("VTOR @%p\n", vtor);
	for (int i = 0; i < nv; i++) {
		vectors[i] = vtor[i];
	}
	*vtor_p = (uint32_t) &vectors[0];
	vtor = (void *)*vtor_p;
	printk("VTOR now @%p\n", vtor);

	/* And hook the SVC call with our own function above, allowing
	 * us direct access to interrupt entry
	 */
	vtor[11] = (int)my_svc;
	printk("vtor[11] == %p (my_svc == %p)\n", (void*)vtor[11], my_svc);

}

ZTEST_SUITE(arm_m_switch, NULL, vector_hijack, NULL, NULL, NULL);
