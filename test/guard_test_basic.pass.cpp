//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#define ABORT_WITH_MESSAGE(...) __builtin_abort()
#include "../src/cxa_guard_impl.h"

using namespace __cxxabiv1;

template <class GuardType, class Impl>
struct Tests {
private:
  Tests() : g{}, impl(&g) {}
  GuardType g;
  Impl impl;

  uint8_t first_byte() {
    uint8_t first = {};
    std::memcpy(&first, &g, 1);
    return first;
  }

  void reset() { g = {}; }

public:
  static void test() {
    Tests tests;
    tests.test_acquire();
    tests.test_abort();
    tests.test_release();
  }

  void test_acquire() {
    {
      reset();
      assert(first_byte() == 0);
      assert(impl.acquire() == INIT_IS_PENDING);
      assert(first_byte() == 0);
    }
    {
      reset();
      assert(first_byte() == 0);
      assert(impl.acquire() == INIT_IS_PENDING);
      impl.release();
      assert(first_byte() == 1);
      assert(impl.acquire() == INIT_IS_DONE);
    }
  }

  void test_release() {
    {
      reset();
      assert(first_byte() == 0);
      assert(impl.acquire() == INIT_IS_PENDING);
      assert(first_byte() == 0);
      impl.release();
      assert(first_byte() == 1);
    }
  }

  void test_abort() {
    {
      reset();
      assert(first_byte() == 0);
      assert(impl.acquire() == INIT_IS_PENDING);
      assert(first_byte() == 0);
      impl.abort();
      assert(first_byte() == 0);
      assert(impl.acquire() == INIT_IS_PENDING);
      assert(first_byte() == 0);
    }
  }
};

struct NopMutex {
  bool lock() {
    assert(!is_locked);
    is_locked = true;
    return false;
  }
  bool unlock() {
    assert(is_locked);
    is_locked = false;
    return false;
  }

private:
  bool is_locked = false;
};
static NopMutex global_nop_mutex = {};

struct NopCondVar {
  bool broadcast() { return false; }
  bool wait(NopMutex&) { return false; }
};
static NopCondVar global_nop_cond = {};

void NopFutexWait(int*, int) { assert(false); }
void NopFutexWake(int*) { assert(false); }
uint32_t MockGetThreadID() { return 0; }

int main() {
  {
#if defined(_LIBCXXABI_HAS_NO_THREADS)
    static_assert(CurrentImplementation == Implementation::NoThreads, "");
    static_assert(
        std::is_same<SelectedImplementation, InitByteNoThreads>::value, "");
#else
    static_assert(CurrentImplementation == Implementation::GlobalLock, "");
    static_assert(
        std::is_same<
            SelectedImplementation,
            InitByteGlobalMutex<LibcppMutex, LibcppCondVar,
                                GlobalStatic<LibcppMutex>::instance,
                                GlobalStatic<LibcppCondVar>::instance>>::value,
        "");
#endif
  }
  {
#if defined(__APPLE__) || defined(__linux__)
    assert(PlatformThreadID);
#endif
    if (+PlatformThreadID) {
      assert(PlatformThreadID() != 0);
      assert(PlatformThreadID() == PlatformThreadID());
    }
  }
  {
    Tests<uint32_t, InitByteNoThreads>::test();
    Tests<uint64_t, InitByteNoThreads>::test();
  }
  {
    using MutexImpl =
        InitByteGlobalMutex<NopMutex, NopCondVar, global_nop_mutex,
                            global_nop_cond, MockGetThreadID>;
    Tests<uint32_t, MutexImpl>::test();
    Tests<uint64_t, MutexImpl>::test();
  }
  {
    using FutexImpl =
        InitByteFutex<&NopFutexWait, &NopFutexWake, &MockGetThreadID>;
    Tests<uint32_t, FutexImpl>::test();
    Tests<uint64_t, FutexImpl>::test();
  }
}
