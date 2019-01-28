//===----------------------------------------------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "potentially_blocking_region_hooks.h"

#ifdef LIBCXXABI_ENABLE_POTENTIALLY_BLOCKING_REGION_HOOKS
extern "C" {
  _LIBCXXABI_WEAK void __libcppabi_potentially_blocking_region_begin(void) {}
  _LIBCXXABI_WEAK void __libcppabi_potentially_blocking_region_end(void) {}
} // extern "C"
#endif // LIBCXXABI_ENABLE_POTENTIALLY_BLOCKING_REGION_HOOKS
