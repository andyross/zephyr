/* Copyright (c) 2022 Google LLC.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <ztest.h>
#include <zephyr.h>
#include <sys/util.h>

#define NUM_THREADS 4
#define STACKSZ (512 + CONFIG_TEST_EXTRA_STACK_SIZE)

struct k_zync zync = K_ZYNC_INITIALIZER(0, true, false, false, 0);
ZTEST_DMEM k_zync_atom_t mod_atom, reset_atom;

struct k_thread wait_threads[NUM_THREADS];
K_THREAD_STACK_ARRAY_DEFINE(wait_stacks, NUM_THREADS, STACKSZ);

ZTEST_DMEM atomic_t awoken_count, awaiting_count;

/* Resets the zync to a test initial-state, returns current config */
static void reset_zync(struct k_zync_cfg *cfg)
{
	struct k_zync_cfg base_cfg = {
		.fair = true,
	};

	k_zync_reset(&zync, &mod_atom);
	k_zync_set_config(&zync, &base_cfg);
	if (cfg) {
		k_zync_get_config(&zync, cfg);
	}
}

static void wait_thread_fn(void *pa, void *pb, void *pc)
{
	int ret;

	atomic_inc(&awaiting_count);
	ret = k_zync(&zync, &mod_atom, NULL, -1, K_FOREVER);

	zassert_equal(ret, 1, "wrong return from k_zync()");
	atomic_dec(&awaiting_count);
	atomic_inc(&awoken_count);
}

static void spawn_wait_thread(int id, bool start)
{
	k_thread_create(&wait_threads[id], wait_stacks[id],
			K_THREAD_STACK_SIZEOF(wait_stacks[id]),
			wait_thread_fn, (void *)(long)id, NULL, NULL,
			0, K_USER | K_INHERIT_PERMS,
			start ? K_NO_WAIT : K_FOREVER);
}

ZTEST_USER(zync_tests, test_zync0_updown)
{
	reset_zync(NULL);

	zassert_true(mod_atom.val == 0, "wrong init val");
	k_zync(&zync, &mod_atom, NULL, 1, K_NO_WAIT);
	zassert_true(mod_atom.val == 1, "val didn't increment");
	k_zync(&zync, &mod_atom, NULL, -1, K_NO_WAIT);
	zassert_true(mod_atom.val == 0, "val didn't decrement");
}

ZTEST_USER(zync_tests, test_zync_downfail)
{
	int32_t t0, t1, ret;

	reset_zync(NULL);

	zassert_true(mod_atom.val == 0, "atom not zero");

	ret = k_zync(&zync, &mod_atom, NULL, -1, K_NO_WAIT);

	zassert_true(ret == -EAGAIN, "wrong return value: %d", ret);
	zassert_true(mod_atom.val == 0, "atom changed unexpectedly");

	k_usleep(1); /* tick align */
	t0 = (int32_t) k_uptime_ticks();
	ret = k_zync(&zync, &mod_atom, NULL, -1, K_TICKS(1));
	t1 = (int32_t) k_uptime_ticks();

	zassert_true(ret == -EAGAIN, "wrong return value: %d", ret);
	zassert_true(mod_atom.val == 0, "atom changed unexpectedly");
	zassert_true(t1 > t0, "timeout didn't elapse");
}

ZTEST_USER(zync_tests, test_zync_updown_n)
{
	const int count = 44, count2 = -14;
	struct k_zync_cfg cfg;

	reset_zync(&cfg);

	k_zync(&zync, &mod_atom, NULL, count, K_NO_WAIT);
	zassert_true(mod_atom.val == count, "wrong atom val");

	k_zync(&zync, &mod_atom, NULL, count2, K_NO_WAIT);
	zassert_true(mod_atom.val == count + count2, "wrong atom val");

#ifdef CONFIG_ZYNC_MAX_VAL
	const int32_t max = 99;

	cfg.max_val = max;
	k_zync_set_config(&zync, &cfg);

	k_zync(&zync, &mod_atom, NULL, 2 * max, K_NO_WAIT);
	zassert_true(mod_atom.val == max, "wrong atom val: %d", mod_atom.val);

	cfg.max_val = 0;
	k_zync_set_config(&zync, &cfg);
#endif

	k_zync_reset(&zync, &mod_atom);
	zassert_true(mod_atom.val == 0, "atom did not reset");
}

ZTEST_USER(zync_tests, test_zync_waiters)
{
	k_zync_reset(&zync, &mod_atom);
	zassert_true(mod_atom.atomic == 0, "atom did not reset");

	awaiting_count = awoken_count = 0;

	for (int i = 0; i < NUM_THREADS; i++) {
		spawn_wait_thread(i, true);
	}

	k_sleep(K_TICKS(1));
	zassert_equal(awoken_count, 0, "someone woke up");
	zassert_equal(awaiting_count, NUM_THREADS, "wrong count of wait threads");

	for (int i = 0; i < NUM_THREADS; i++) {
		k_zync(&zync, &mod_atom, NULL, 1, K_NO_WAIT);
		k_sleep(K_TICKS(1));
		zassert_equal(awoken_count, i + 1, "wrong woken count");
		zassert_equal(awaiting_count, NUM_THREADS - 1 - i,
			      "wrong woken count");
	}

	k_sleep(K_TICKS(1));
}

ZTEST_USER(zync_tests, test_zync_wake_all)
{
	k_zync_reset(&zync, &mod_atom);
	zassert_true(mod_atom.atomic == 0, "atom did not reset");

	awaiting_count = awoken_count = 0;

	for (int i = 0; i < NUM_THREADS; i++) {
		spawn_wait_thread(i, true);
	}

	k_sleep(K_TICKS(1));
	zassert_equal(awoken_count, 0, "someone woke up");
	zassert_equal(awaiting_count, NUM_THREADS, "wrong count of wait threads");

	k_zync(&zync, &mod_atom, NULL, NUM_THREADS + 1, K_NO_WAIT);
	k_sleep(K_TICKS(1));
	zassert_equal(awoken_count, NUM_THREADS, "wrong woken count");
	zassert_equal(awaiting_count, 0, "wrong woken count");
	zassert_equal(mod_atom.val, 1, "wrong atom value");
}

ZTEST_USER(zync_tests, test_reset_atom)
{
	int32_t ret;

	mod_atom.val = 0;
	reset_atom.val = 2;

	/* reset_atom != mod_atom */
	ret = k_zync(&zync, &mod_atom, &reset_atom, 1, K_NO_WAIT);
	zassert_equal(ret, 1, "wrong return value");
	zassert_equal(reset_atom.val, 1, "wrong reset atom value");
	zassert_equal(mod_atom.val, 1, "atom value didn't increment");

	k_zync_reset(&zync, &mod_atom);

	/* reset_atom == mod_atom (note different return!) */
	ret = k_zync(&zync, &mod_atom, &mod_atom, 1, K_NO_WAIT);
	zassert_equal(ret, 0, "wrong return value");
	zassert_equal(mod_atom.val, 0, "atom value didn't remain zero");
}

/* Not userspace: whiteboxes zync object */
ZTEST(zync_tests, test_zync_config)
{
	struct k_zync_cfg cfg;

	k_zync_get_config(&zync, &cfg);
	k_zync_reset(&zync, &mod_atom);

	cfg.fair = false;
	IF_ENABLED(CONFIG_ZYNC_PRIO_BOOST,
		   (cfg.prio_boost = true));
	IF_ENABLED(CONFIG_ZYNC_MAX_VAL,
		   (cfg.max_val = 3));
	k_zync_set_config(&zync, &cfg);

	zassert_equal(zync.cfg.fair, false, "wrong fair");
	IF_ENABLED(CONFIG_ZYNC_PRIO_BOOST,
		   (zassert_equal(zync.cfg.prio_boost, true,
				  "wrong prio_boost")));
	IF_ENABLED(CONFIG_ZYNC_MAX_VAL,
		   (zassert_equal(zync.cfg.max_val, 3,
				  "wrong max_val")));

	cfg.fair = true;
	IF_ENABLED(CONFIG_ZYNC_PRIO_BOOST,
		   (cfg.prio_boost = false));
	IF_ENABLED(CONFIG_ZYNC_MAX_VAL,
		   (cfg.max_val = 0));
	k_zync_set_config(&zync, &cfg);

	zassert_equal(zync.cfg.fair, true, "wrong fair");
	IF_ENABLED(CONFIG_ZYNC_PRIO_BOOST,
		   (zassert_equal(zync.cfg.prio_boost, false,
				  "wrong prio_boost")));
	IF_ENABLED(CONFIG_ZYNC_MAX_VAL,
		   (zassert_equal(zync.cfg.max_val, K_ZYNC_ATOM_VAL_MAX,
				  "wrong max val")));
}

/* 1cpu because to exercise "fairness", we need to test for preemption
 * of the current thread, which is impossible if another cpu can pick
 * up the thread that should preempt us.
 */
ZTEST(zync_tests_1cpu, test_fair)
{

	struct k_zync_cfg cfg;

	for (int pass = 0; pass < 2; pass++) {
		bool is_fair = pass == 0;

		reset_zync(&cfg);

		cfg.fair = is_fair;
		k_zync_set_config(&zync, &cfg);

		awaiting_count = awoken_count = 0;
		spawn_wait_thread(0, true);

		/* Make sure we're lower priority and preemptible */
		k_thread_priority_set(k_current_get(),
				      1 + k_thread_priority_get(&wait_threads[0]));
		__ASSERT_NO_MSG(k_thread_priority_get(k_current_get()) >= 0);

		/* Make sure it blocked */
		zassert_equal(awoken_count, 0, "thread woke up");
		zassert_equal(awaiting_count, 1, "thread didn't run");

		/* Wake it up, see if we're preempted */
		k_zync(&zync, &mod_atom, NULL, 1, K_NO_WAIT);

		if (is_fair) {
			zassert_equal(awoken_count, 1, "thread didn't run");
		} else {
			zassert_equal(awoken_count, 0, "thread ran unexpectedly");
		}

		k_sleep(K_TICKS(1)); /* let thread terminate */

		zassert_equal(awoken_count, 1, "thread didn't resume");
	}
}

/* Not userspace: increases wait_threads[0] priority */
ZTEST(zync_tests, test_prio_boost)
{
	struct k_zync_cfg cfg;

	reset_zync(&cfg);

	if (!IS_ENABLED(CONFIG_ZYNC_PRIO_BOOST)) {
		ztest_test_skip();
	}

	IF_ENABLED(CONFIG_ZYNC_PRIO_BOOST,
		   (cfg.prio_boost = true));
	k_zync_set_config(&zync, &cfg);

	int curr_prio = k_thread_priority_get(k_current_get());
	int thread_prio = curr_prio - 1;

	/* "Take the lock" */
	mod_atom.val = 1;
	k_zync(&zync, &mod_atom, NULL, -1, K_NO_WAIT);

	zassert_equal(k_thread_priority_get(k_current_get()), curr_prio,
		      "thread priority changed unexpectedly");

	spawn_wait_thread(0, false);
	k_thread_priority_set(&wait_threads[0], thread_prio);
	k_thread_start(&wait_threads[0]);
	k_sleep(K_TICKS(1));

	/* We should get its priority */
	zassert_equal(k_thread_priority_get(k_current_get()), thread_prio,
		      "thread priority didn't boost");

	/* Wake it up, check our priority resets */
	k_zync(&zync, &mod_atom, NULL, 1, K_NO_WAIT);

	zassert_equal(k_thread_priority_get(k_current_get()), curr_prio,
		      "thread priority wasn't restored");
}

ZTEST_USER(zync_tests, test_recursive)
{
	const int lock_count = 16;
	struct k_zync_cfg cfg;

	if (!IS_ENABLED(CONFIG_ZYNC_RECURSIVE)) {
		ztest_test_skip();
	}

	reset_zync(&cfg);
	IF_ENABLED(CONFIG_ZYNC_RECURSIVE,
		   (cfg.recursive = true));
	k_zync_set_config(&zync, &cfg);

	mod_atom.val = 1; /* start "unlocked" */

	for (int i = 0; i < lock_count; i++) {
		k_zync(&zync, &mod_atom, NULL, -1, K_NO_WAIT);
		zassert_equal(mod_atom.val, 0, "recursive zync didn't lock");
	}

	for (int i = 0; i < (lock_count - 1); i++) {
		k_zync(&zync, &mod_atom, NULL, 1, K_NO_WAIT);
		zassert_equal(mod_atom.val, 0, "recursive zync unlocked early");
	}

	k_zync(&zync, &mod_atom, NULL, 1, K_NO_WAIT);
	zassert_equal(mod_atom.val, 1, "recursive zync didn't unlock");
}

}

static void *suite_setup(void)
{
	z_object_init(&zync);
	k_object_access_grant(&zync, k_current_get());
	for (int i = 0; i < NUM_THREADS; i++) {
		k_object_access_grant(&wait_threads[i], k_current_get());
		k_object_access_grant(wait_stacks[i], k_current_get());
	}

	return NULL;
}

ZTEST_SUITE(zync_tests, NULL, suite_setup, NULL, NULL, NULL);

ZTEST_SUITE(zync_tests_1cpu, NULL, NULL,
	    ztest_simple_1cpu_before, ztest_simple_1cpu_after, NULL);
