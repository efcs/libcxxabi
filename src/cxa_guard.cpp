//===---------------------------- cxa_guard.cpp ---------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "__cxxabi_config.h"
#include "include/cxa_guard_impl.h"

/*
    This implementation must be careful to not call code external to this file
    which will turn around and try to call __cxa_guard_acquire reentrantly.
    For this reason, the headers of this file are as restricted as possible.
    Previous implementations of this code for __APPLE__ have used
    std::__libcpp_mutex_lock and the abort_message utility without problem. This
    implementation also uses std::__libcpp_condvar_wait which has tested
    to not be a problem.
*/

namespace __cxxabiv1 {

extern "C"
{

using guard_type = std::conditional<ABI::Current == ABI::ARM, uint32_t, uint64_t>::type;

_LIBCXXABI_FUNC_VIS int __cxa_guard_acquire(guard_type* raw_guard_object) {
  CurrentImplementation imp(raw_guard_object);
  return static_cast<int>(imp.acquire());
}

_LIBCXXABI_FUNC_VIS void __cxa_guard_release(guard_type *raw_guard_object) {
  CurrentImplementation imp(raw_guard_object);
  imp.release();
}

_LIBCXXABI_FUNC_VIS void __cxa_guard_abort(guard_type *raw_guard_object) {
  CurrentImplementation imp(raw_guard_object);
  imp.abort();
}
}  // extern "C"

}  // __cxxabiv1
