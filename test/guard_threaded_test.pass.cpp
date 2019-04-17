//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

// UNSUPPORTED: c++98, c++03
// UNSUPPORTED: libcxxabi-no-threads

#define ABORT_WITH_MESSAGE(...) __builtin_abort()
#include "../src/cxa_guard_impl.h"
#include <unordered_map>
#include <thread>


using namespace __cxxabiv1;

enum class InitResult {
  COMPLETE,
  PERFORMED,
  WAITED,
  ABORTED
};
constexpr InitResult COMPLETE = InitResult::COMPLETE;
constexpr InitResult PERFORMED = InitResult::PERFORMED;
constexpr InitResult WAITED = InitResult::WAITED;
constexpr InitResult ABORTED = InitResult::ABORTED;

template <class Impl, class GuardType, class Init>
InitResult check_guard(GuardType *g, Init init) {
  uint8_t *first_byte = reinterpret_cast<uint8_t*>(g);
  if (std::__libcpp_atomic_load(first_byte, std::_AO_Acquire) == 0) {
    Impl impl(g);
    if (impl.cxa_guard_acquire() == INIT_IS_PENDING) {
#ifndef LIBCXXABI_HAS_NO_EXCEPTIONS
      try {
#endif
        init();
        impl.cxa_guard_release();
        return PERFORMED;
#ifndef LIBCXXABI_HAS_NO_EXCEPTIONS
      } catch (...) {
        impl.cxa_guard_abort();
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

template <bool HasFutex = +PlatformFutexWait != nullptr>
void test_futex() {
   using TestFutex = SelectImplementation<Implementation::Futex>::type;
   Tests<uint32_t, TestFutex>::test();
   Tests<uint64_t, TestFutex>::test();
}
template <> void test_futex<false>() {}


int main() {
  {
    using TestMutex = SelectImplementation<Implementation::GlobalLock>::type;
    Tests<uint32_t, TestMutex>::test();
    Tests<uint64_t, TestMutex>::test();
  }
  {
    test_futex();
  }
}
