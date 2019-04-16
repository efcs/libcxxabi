//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//


#define ABORT_WITH_MESSAGE(...) __builtin_abort()
#include "../src/cxa_guard_impl.h"
#include <unordered_map>
#include <thread>


using namespace __cxxabiv1;


enum InitResult {
  COMPLETE,
  PERFORMED,
  WAITED,
  ABORTED
};

template <class Impl, class GuardType, class Init>
InitResult check_guard(GuardType *g, Init init) {
  uint8_t *first_byte = (uint8_t*)g;
  if (std::__libcpp_atomic_load(first_byte, std::_AO_Acquire) == 0) {
    Impl impl(g);
    if (impl.acquire() == INIT_IS_PENDING) {
#ifndef LIBCXXABI_HAS_NO_EXCEPTIONS
      try {
#endif
        init();
        impl.release();
        return PERFORMED;
#ifndef LIBCXXABI_HAS_NO_EXCEPTIONS
      } catch (...) {
        impl.abort();
        return ABORTED;
      }
#endif
    }
    return WAITED;
  }
  return COMPLETE;
}


template <class GuardType, class Impl>
struct Tests {
private:
  Tests() = delete;

public:
  static void test() {
    GuardType g;
    check_guard<Impl>(&g, []() {});
  }

};

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunreachable-code"
#endif
int main() {
  {
    using TestMutex = SelectImplementation<Implementation::GlobalLock>::type;
    Tests<uint32_t, TestMutex>::test();
    Tests<uint64_t, TestMutex>::test();
  }
  if (+PlatformFutexWait)
  {
    using TestFutex = SelectImplementation<Implementation::Futex>::type;
    Tests<uint32_t, TestFutex>::test();
    Tests<uint64_t, TestFutex>::test();
  }
}
#ifdef __clang__
#pragma clang diagnostic pop
#endif
