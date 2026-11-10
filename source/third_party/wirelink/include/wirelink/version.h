/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WIRELINK_VERSION_H_
#define WIRELINK_VERSION_H_

#define WIRELINK_VERSION_MAJOR 0
#define WIRELINK_VERSION_MINOR 8
#define WIRELINK_VERSION_PATCH 0
#define WIRELINK_VERSION_STRING "0.8.0"

#define WIRELINK_PROTOCOL_VERSION 1

/*
 * Generated-code contract this core accepts. WLC declares the same pair in
 * every generated runtime. The predicate encodes the compatibility rule: the
 * contract must match exactly before 1.0, and from 1.0 on a generated minor at
 * or below this core's minor is accepted.
 */
#define WIRELINK_CODEGEN_CONTRACT_MAJOR 0
#define WIRELINK_CODEGEN_CONTRACT_MINOR 8

#if WIRELINK_CODEGEN_CONTRACT_MAJOR == 0
#  define WIRELINK_CODEGEN_CONTRACT_ACCEPTS(major, minor) \
     ((major) == WIRELINK_CODEGEN_CONTRACT_MAJOR && \
      (minor) == WIRELINK_CODEGEN_CONTRACT_MINOR)
#else
#  define WIRELINK_CODEGEN_CONTRACT_ACCEPTS(major, minor) \
     ((major) == WIRELINK_CODEGEN_CONTRACT_MAJOR && \
      (minor) <= WIRELINK_CODEGEN_CONTRACT_MINOR)
#endif

#endif /* WIRELINK_VERSION_H_ */
