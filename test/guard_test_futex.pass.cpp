#define ABORT_WITH_MESSAGE(...) __builtin_abort()
#include "../src/include/cxa_guard_impl.h"
#include <unordered_map>

using namespace __cxxabiv1;

#if 0
enum InitResult {
  COMPLETE,
  PERFORMED,
  WAITED,
  ABORTED
};

template <class Impl, class GuardType, class Init>
InitResult check_guard(GuardType *g, Init init) {
  uint8_t *byte = (uint8_t*)g;
  if (std::__libcpp_atomic_load(g, std::_AO_Acquire) == 0) {
    Impl impl(g);
    if (impl.acquire()) {
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
#endif

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
struct NopCondVar {
  bool broadcast() { return false; }
  bool wait(NopMutex&) { return false; }
};
void NopFutexWait(int*, int) { assert(false); }
void NopFutexWake(int*) { assert(false); }

int main() {
  {
    Tests<uint32_t, NoThreadsImpl>::test();
    Tests<uint64_t, NoThreadsImpl>::test();
  }
  {
    using MutexImpl = GlobalMutexImpl<NopMutex, NopCondVar>;
    Tests<uint32_t, MutexImpl>::test();
    Tests<uint64_t, MutexImpl>::test();
  }
  {
    using FutexImpl = ::FutexImpl<&NopFutexWait, &NopFutexWake>;
    Tests<uint32_t, FutexImpl>::test();
    Tests<uint64_t, FutexImpl>::test();
  }
}
