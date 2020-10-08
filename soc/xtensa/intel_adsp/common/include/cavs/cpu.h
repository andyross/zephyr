/* SPDX-License-Identifier: Apache-2.0
 *
 * Copyright(c) 2019 Intel Corporation. All rights reserved.
 *
 * Author: Bartosz Kokoszko <bartoszx.kokoszko@linux.intel.com>
 */

/**
 * \file cavs/lib/cpu.h
 * \brief DSP parameters, common for cAVS platforms.
 */

#ifndef __CAVS_CPU_H__
#define __CAVS_CPU_H__

#ifndef PLATFORM_CORE_COUNT
/** \brief Number of available DSP cores (conf. by kconfig) */
#ifndef CONFIG_SOF
#define PLATFORM_CORE_COUNT	CONFIG_CORE_COUNT
#else
#define PLATFORM_CORE_COUNT	1
#endif

/** \brief Id of master DSP core */
#define PLATFORM_MASTER_CORE_ID	0
#endif

#endif /* __CAVS_CPU_H__ */
