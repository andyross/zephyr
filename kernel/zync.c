/* Copyright (c) 2022 Google LLC.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <zephyr/sys/zync.h>
#include <ksched.h>
#include <zephyr/wait_q.h>
#include <zephyr/syscall_handler.h>

/* Sets the priority of the zync owner (if it exists) to the highest
 * logical priority of the pri argument, the thread's base priority,
 * and the highest priority waiting thread
 */
static void prio_boost(struct k_zync *zync, int pri)
{
#ifdef CONFIG_ZYNC_PRIO_BOOST
	if (zync->cfg.prio_boost && zync->owner != NULL) {
		struct k_thread *th = z_waitq_head(&zync->waiters);

		pri = MIN(pri, zync->owner->base.zync_prio);
		if (th != NULL) {
			pri = MIN(pri, th->base.prio);
		}
		z_set_prio(zync->owner, pri);
	}
#endif
}

static void prio_boost_reset(struct k_zync *zync)
{
#ifdef CONFIG_ZYNC_PRIO_BOOST
	if (zync->cfg.prio_boost) {
		z_set_prio(_current, _current->base.zync_prio);
	}
#endif
}

static void set_owner(struct k_zync *zync)
{
#ifdef Z_ZYNC_OWNER
# ifdef CONFIG_ZYNC_PRIO_BOOST
	if (zync->cfg.prio_boost) {
		if (zync->owner != NULL) {
			z_set_prio(zync->owner, zync->owner->base.zync_prio);
		}
		_current->base.zync_prio = _current->base.prio;
	}
# endif
	zync->owner = _current;
#endif
}

static inline int32_t modclamp(struct k_zync *zync, int32_t mod)
{
	int32_t max = K_ZYNC_ATOM_VAL_MAX;

#ifdef CONFIG_ZYNC_MAX_VAL
	// FIXME: don't check vs. 0 at runtime, clamp when set!
	if (zync->cfg.max_val != 0) {
		max = MIN(max, zync->cfg.max_val);
	}
#endif
	return CLAMP(mod, 0, max);
}

void z_impl_k_zync_set_config(struct k_zync *zync,
			      const struct k_zync_cfg *cfg)
{
	k_spinlock_key_t key = k_spin_lock(&zync->lock);

	zync->cfg = *cfg;
	IF_ENABLED(CONFIG_ZYNC_MAX_VAL,
		   (zync->cfg.max_val = Z_ZYNC_MVCLAMP(zync->cfg.max_val)));
	k_spin_unlock(&zync->lock, key);
}

void z_impl_k_zync_get_config(struct k_zync *zync,
			      struct k_zync_cfg *cfg)
{
	k_spinlock_key_t key = k_spin_lock(&zync->lock);

	*cfg = zync->cfg;
	k_spin_unlock(&zync->lock, key);
}

void z_impl_k_zync_init(struct k_zync *zync, k_zync_atom_t *atom,
			struct k_zync_cfg *cfg)
{
	memset(zync, 0, sizeof(*zync));
	k_zync_set_config(zync, cfg);
	atom->val = cfg->atom_init;
#ifdef CONFIG_POLL
	zync->pollable = (cfg->atom_init != 0);
#endif
}

int32_t z_impl_k_zync(struct k_zync *zync, k_zync_atom_t *mod_atom,
		      k_zync_atom_t *reset_atom, int32_t mod, k_timeout_t timeout)
{
	k_spinlock_key_t key = k_spin_lock(&zync->lock);
	bool resched = false, nowait, must_pend;
	int32_t delta = 0, delta2 = 0, val0 = 0, val1 = 0, pendret = 0, woken;

#ifdef CONFIG_ZYNC_RECURSIVE
	if (zync->cfg.recursive && mod > 0) {
		if (_current != zync->owner) {
			if (IS_ENABLED(CONFIG_ZYNC_VALIDATE)) {
				__ASSERT(0, "unlocking unowned recursive zync");
			}
			/* Weird returns are from old k_mutex */
			pendret = zync->owner == NULL ? -EINVAL : -EPERM;
			k_spin_unlock(&zync->lock, key);
			return pendret;
		}
		delta = MIN(mod, zync->rec_count);
		zync->rec_count -= delta;
		mod -= delta;
	}
#endif

	K_ZYNC_ATOM_SET(mod_atom) {
		val0 = old_atom.val;
		val1 = modclamp(zync, val0 + mod);
		delta = val1 - val0;
		new_atom.val = (mod_atom == reset_atom) ? 0 : val1;
		new_atom.waiters = mod < 0 && delta != mod;
	}

	nowait = K_TIMEOUT_EQ(timeout, Z_TIMEOUT_NO_WAIT);
	must_pend = mod < 0 && mod != delta;

#ifdef CONFIG_ZYNC_RECURSIVE
	if (must_pend && zync->cfg.recursive && _current == zync->owner) {
		zync->rec_count += -(mod - delta);
		mod = 0;
		must_pend = false;
	}
#endif

#ifdef Z_ZYNC_OWNER
	if (val1 > 0) {
		zync->owner = NULL;
	}
#endif

	if (delta > 0) {
		prio_boost_reset(zync);
	}

#ifdef CONFIG_POLL
	if (delta > 0 && val0 == 0) {
		z_handle_obj_poll_events(&zync->poll_events, K_POLL_STATE_ZYNC);
		resched = true;
	}
	zync->pollable = (val1 != 0);
#endif

	Z_WAIT_Q_LAZY_INIT(&zync->waiters);
	for (woken = 0; woken < delta; woken++) {
		if (!z_sched_wake(&zync->waiters, 0, NULL)) {
			break;
		}
		resched = true;
	}

	/* Old condvar API wants the count of threads woken as the return value */
	if (delta >= 0 && mod_atom == reset_atom) {
		delta = woken;
	}

	if (resched) {
		K_ZYNC_ATOM_SET(mod_atom) {
			new_atom.waiters = z_waitq_head(&zync->waiters) != NULL;
		}
	}

	if (reset_atom != NULL) {
		uint32_t newval = reset_atom == mod_atom ? 0 : 1;

		if (IS_ENABLED(CONFIG_ZYNC_VALIDATE)) {
			__ASSERT(newval == 0 || newval != reset_atom->val,
				 "noop zync reset (mislocked condvar?)");
		}
		K_ZYNC_ATOM_SET(reset_atom) {
			new_atom.val = newval;
		}
	}

	if (must_pend && !nowait) {
		prio_boost(zync, _current->base.prio);
		pendret = z_pend_curr(&zync->lock, key, &zync->waiters, timeout);
		key = k_spin_lock(&zync->lock);

		mod -= delta;
		K_ZYNC_ATOM_SET(mod_atom) {
			new_atom.val = modclamp(zync, old_atom.val + mod);
			delta2 = new_atom.val - old_atom.val;
		}
		delta += delta2;
	} else if (must_pend && nowait) {
		pendret = -EAGAIN;
	}

	if (delta < 0) {
		set_owner(zync);
	}

	if (resched && zync->cfg.fair) {
		z_reschedule(&zync->lock, key);
	} else {
		k_spin_unlock(&zync->lock, key);
	}
	return pendret < 0 ? pendret : abs(delta);
}

void z_impl_k_zync_reset(struct k_zync *zync, k_zync_atom_t *atom)
{
	k_spinlock_key_t key = k_spin_lock(&zync->lock);

	atom->val = zync->cfg.atom_init;

	while (z_waitq_head(&zync->waiters)) {
		z_sched_wake(&zync->waiters, -EAGAIN, NULL);
	}

	IF_ENABLED(CONFIG_ZYNC_RECURSIVE, (zync->rec_count = 0));
	IF_ENABLED(Z_ZYNC_OWNER,          (zync->owner = NULL));

	k_spin_unlock(&zync->lock, key);
}

#ifdef CONFIG_ZYNC_USERSPACE_COMPAT
int32_t z_impl_z_pzync(struct k_zync *zync, int32_t mod, k_timeout_t timeout)
{
	return k_zync(zync, &zync->atom, NULL, mod, timeout);
}

uint32_t z_impl_z_zync_atom_val(struct k_zync *zync)
{
	return zync->atom.val;
}
#endif

#ifdef CONFIG_USERSPACE

void z_vrfy_k_zync_set_config(struct k_zync *zync, const struct k_zync_cfg *cfg)
{
	Z_OOPS(Z_SYSCALL_OBJ_INIT(zync, K_OBJ_ZYNC));
        Z_OOPS(Z_SYSCALL_MEMORY_READ(cfg, sizeof(*cfg)));
	z_impl_k_zync_set_config(zync, cfg);
}
#include <syscalls/k_zync_set_config_mrsh.c>

void z_vrfy_k_zync_get_config(struct k_zync *zync, struct k_zync_cfg *cfg)
{
	Z_OOPS(Z_SYSCALL_OBJ_INIT(zync, K_OBJ_ZYNC));
        Z_OOPS(Z_SYSCALL_MEMORY_WRITE(cfg, sizeof(*cfg)));
	z_impl_k_zync_get_config(zync, cfg);
}
#include <syscalls/k_zync_get_config_mrsh.c>

static void chk_atom(struct k_zync *zync, k_zync_atom_t *atom)
{
#ifdef CONFIG_ZYNC_USERSPACE_COMPAT
	if (atom == &zync->atom) {
		return;
	}
#endif
	Z_OOPS(Z_SYSCALL_MEMORY_WRITE(atom, sizeof(*atom)));
}

void z_vrfy_k_zync_init(struct k_zync *zync, k_zync_atom_t *atom,
			struct k_zync_cfg *cfg)
{
	Z_OOPS(Z_SYSCALL_OBJ_INIT(zync, K_OBJ_ZYNC));
	chk_atom(zync, atom);
	Z_OOPS(Z_SYSCALL_MEMORY_READ(cfg, sizeof(*cfg)));
	z_impl_k_zync_init(zync, atom, cfg);
	z_object_init(zync);
}
#include <syscalls/k_zync_init_mrsh.c>

int32_t z_vrfy_k_zync(struct k_zync *zync, k_zync_atom_t *mod_atom,
		      k_zync_atom_t *reset_atom, int32_t mod, k_timeout_t timeout)
{
	Z_OOPS(Z_SYSCALL_OBJ_INIT(zync, K_OBJ_ZYNC));
	chk_atom(zync, mod_atom);
	if (reset_atom != NULL) {
		int ret = -1;

#ifdef CONFIG_ZYNC_USERSPACE_COMPAT
		/* may be an atom field of a valid zync */
		ret = Z_SYSCALL_OBJ_INIT(CONTAINER_OF(reset_atom, struct k_zync, atom),
					 K_OBJ_ZYNC);
#endif
		if (ret) {
			ret = Z_SYSCALL_MEMORY_WRITE(reset_atom, sizeof(*reset_atom));
		}
		Z_OOPS(ret);
	}
	return z_impl_k_zync(zync, mod_atom, reset_atom, mod, timeout);
}
#include <syscalls/k_zync_mrsh.c>

void z_vrfy_k_zync_reset(struct k_zync *zync, k_zync_atom_t *atom)
{
	Z_OOPS(Z_SYSCALL_OBJ_INIT(zync, K_OBJ_ZYNC));
	chk_atom(zync, atom);
	z_impl_k_zync_reset(zync, atom);
}
#include <syscalls/k_zync_reset_mrsh.c>

#ifdef CONFIG_ZYNC_USERSPACE_COMPAT
int32_t z_vrfy_z_pzync(struct k_zync *zync, int32_t mod, k_timeout_t timeout)
{
        Z_OOPS(Z_SYSCALL_OBJ_INIT(zync, K_OBJ_ZYNC));
	return z_impl_z_pzync(zync, mod, timeout);
}
#include <syscalls/z_pzync_mrsh.c>

uint32_t z_vrfy_z_zync_atom_val(struct k_zync *zync)
{
	Z_OOPS(Z_SYSCALL_OBJ_INIT(zync, K_OBJ_ZYNC));
	return z_impl_z_zync_atom_val(zync);
}
#include <syscalls/z_zync_atom_val_mrsh.c>
#endif

#endif /* CONFIG_USERSPACE */
