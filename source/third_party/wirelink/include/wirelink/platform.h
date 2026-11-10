/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WIRELINK_PLATFORM_H
#define WIRELINK_PLATFORM_H
#include "wirelink/environment.h"
#ifdef __cplusplus
extern "C" {
#endif
/* Optional native platform target: Wirelink::platform on desktop,
 * CONFIG_WIRELINK_PLATFORM on Zephyr. No threads, global mutable provider or
 * heap owned by Wirelink. Configuration is cheap; identity acquisition happens
 * at endpoint initialization. Failure never falls back to a weak seed. */
wl_environment_t wl_platform_environment(void);
#ifdef __cplusplus
}
#endif
#endif
