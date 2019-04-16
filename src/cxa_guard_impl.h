//===----------------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
#ifndef LIBCXXABI_SRC_INCLUDE_CXA_GUARD_IMPL_H
#define LIBCXXABI_SRC_INCLUDE_CXA_GUARD_IMPL_H

#include "__cxxabi_config.h"
#include "include/atomic_support.h"
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/types.h>

#ifndef ABORT_WITH_MESSAGE
#include "abort_message.h"
#define ABORT_WITH_MESSAGE(...) ::abort_message(__VA_ARGS__)
#endif

#include <__threading_support>

#define DISALLOW_COPY(T)                                                       \
  T(T const&) = delete;                                                        \
  T& operator=(T const&) = delete

namespace __cxxabiv1 {

namespace {


enum class AcquireResult {
  INIT_IS_DONE,
  INIT_IS_PENDING,
};
constexpr AcquireResult INIT_IS_DONE = AcquireResult::INIT_IS_DONE;
constexpr AcquireResult INIT_IS_PENDING = AcquireResult::INIT_IS_PENDING;

#if defined(__APPLE__) && defined(_LIBCPP_HAS_THREAD_API_PTHREAD)
uint32_t PlatformThreadID() {
  static_assert(sizeof(mach_port_t) == sizeof(uint32_t), "");
  return static_cast<uint32_t>(
      pthread_mach_thread_np(std::__libcpp_thread_get_current_id()));
}
#elif defined(SYS_gettid) && defined(_LIBCPP_HAS_THREAD_API_PTHREAD)
uint32_t PlatformThreadID() {
  static_assert(sizeof(pid_t) == sizeof(uint32_t), "");
  return static_cast<uint32_t>(syscall(SYS_gettid));
}
#else
constexpr uint32_t (*PlatformThreadID)() = nullptr;
#endif

//===----------------------------------------------------------------------===//


static constexpr uint8_t COMPLETE_BIT = (1 << 0);
static constexpr uint8_t PENDING_BIT = (1 << 1);
static constexpr uint8_t WAITING_BIT = (1 << 2);

template <class Derived>
struct GuardBase {
  GuardBase() = delete;
  GuardBase(GuardBase const&) = delete;
  GuardBase& operator=(GuardBase const&) = delete;

  explicit GuardBase(uint32_t* g)
      : base_address(g), guard_byte_address(reinterpret_cast<uint8_t*>(g)),
        init_byte_address(reinterpret_cast<uint8_t*>(g) + 1),
        thread_id_address(nullptr), has_thread_id_checking(false) {}

  explicit GuardBase(uint64_t* g)
      : base_address(g), guard_byte_address(reinterpret_cast<uint8_t*>(g)),
        init_byte_address(reinterpret_cast<uint8_t*>(g) + 1),
        thread_id_address(reinterpret_cast<uint32_t*>(g) + 1),
        has_thread_id_checking(Derived::GetThreadID != nullptr) {}

public:
  AcquireResult acquire() {
    AtomicInt<uint8_t> guard_byte(guard_byte_address);
    if (guard_byte.load(std::_AO_Acquire) == COMPLETE_BIT)
      return INIT_IS_DONE;
    return derived()->acquire_init_byte();
  }

  void release() {
    AtomicInt<uint8_t> guard_byte(guard_byte_address);
    guard_byte.store(COMPLETE_BIT, std::_AO_Release);
    derived()->release_init_byte();
  }

  void abort() { derived()->abort_init_byte(); }

public:
  void* const base_address;
  uint8_t* const guard_byte_address;
  uint8_t* const init_byte_address;
  uint32_t* const thread_id_address;

protected:
  void check_thread_id_for_deadlock() {
    if (has_thread_id_checking) {
      AtomicInt<uint32_t> thread_id(thread_id_address);
      if (thread_id.load(std::_AO_Relaxed) == current_thread_id()) {
        ABORT_WITH_MESSAGE("__cxa_guard_acquire detected deadlock");
      }
    }
  }

  void clear_thread_id() {
    if (has_thread_id_checking) {
      AtomicInt<uint32_t> thread_id(thread_id_address);
      thread_id.store(0, std::_AO_Relaxed);
    }
  }

  void store_thread_id() {
    if (has_thread_id_checking) {
      AtomicInt<uint32_t> thread_id(thread_id_address);
      thread_id.store(current_thread_id(), std::_AO_Relaxed);
    }
  }

private:
  uint32_t current_thread_id() {
    if (has_thread_id_checking && !is_thread_id_cached) {
      current_thread_id_cache = Derived::GetThreadID();
      is_thread_id_cached = true;
    }
    return current_thread_id_cache;
  }

  const bool has_thread_id_checking;
  bool is_thread_id_cached = false;
  uint32_t current_thread_id_cache = 0;

private:
  Derived* derived() { return static_cast<Derived*>(this); }
};

//===----------------------------------------------------------------------===//
//
//===----------------------------------------------------------------------===//

struct InitByteNoThreads : GuardBase<InitByteNoThreads> {
  using GuardBase::GuardBase;

  static constexpr uint32_t (*GetThreadID)() = nullptr;

  AcquireResult acquire_init_byte() {
    if (*init_byte_address == COMPLETE_BIT)
      return INIT_IS_DONE;
    *init_byte_address = PENDING_BIT;
    return INIT_IS_PENDING;
  }

  void release_init_byte() { *init_byte_address = COMPLETE_BIT; }
  void abort_init_byte() { *init_byte_address = 0; }
};

//===----------------------------------------------------------------------===//
//
//===----------------------------------------------------------------------===//

struct LibcppMutex;
struct LibcppCondVar;

#ifndef _LIBCXXABI_HAS_NO_THREADS
struct LibcppMutex {
  LibcppMutex() = default;
  LibcppMutex(LibcppMutex const&) = delete;
  LibcppMutex& operator=(LibcppMutex const&) = delete;

  bool lock() { return std::__libcpp_mutex_lock(&mutex); }
  bool unlock() { return std::__libcpp_mutex_unlock(&mutex); }

private:
  friend struct LibcppCondVar;
  std::__libcpp_mutex_t mutex = _LIBCPP_MUTEX_INITIALIZER;
};

struct LibcppCondVar {
  LibcppCondVar() = default;
  LibcppCondVar(LibcppCondVar const&) = delete;
  LibcppCondVar& operator=(LibcppCondVar const&) = delete;

  bool wait(LibcppMutex& mut) {
    return std::__libcpp_condvar_wait(&cond, &mut.mutex);
  }
  bool broadcast() { return std::__libcpp_condvar_broadcast(&cond); }

private:
  std::__libcpp_condvar_t cond = _LIBCPP_CONDVAR_INITIALIZER;
};
#endif // !defined(_LIBCXXABI_HAS_NO_THREADS)

template <class Mutex, class CondVar, Mutex& global_mutex, CondVar& global_cond,
          uint32_t (*GetThreadIDArg)() = PlatformThreadID>
struct InitByteGlobalMutex
    : GuardBase<InitByteGlobalMutex<Mutex, CondVar, global_mutex, global_cond,
                                    GetThreadIDArg>> {
  using BaseT = GuardBase<InitByteGlobalMutex>;
  using BaseT::BaseT;

  using BaseT::init_byte_address;

  static constexpr uint32_t (*GetThreadID)() = GetThreadIDArg;

  AcquireResult acquire_init_byte() {
    LockGuard g("__cxa_guard_acquire");
    if (*init_byte_address & PENDING_BIT) {
      this->check_thread_id_for_deadlock();
    }
    while (*init_byte_address & PENDING_BIT) {
      *init_byte_address |= WAITING_BIT;
      global_cond.wait(global_mutex);
    }
    if (*init_byte_address)
      return INIT_IS_DONE;
    this->store_thread_id();
    *init_byte_address = PENDING_BIT;
    return INIT_IS_PENDING;
  }

  void release_init_byte() {
    LockGuard g("__cxa_guard_release");
    bool has_waiting = *init_byte_address & WAITING_BIT;
    *init_byte_address = COMPLETE_BIT;
    if (has_waiting)
      g.release_and_broadcast();
  }

  void abort_init_byte() {
    LockGuard g("__cxa_guard_acquire");
    bool has_waiting = *init_byte_address & WAITING_BIT;
    *init_byte_address = 0;
    this->clear_thread_id();
    if (has_waiting)
      g.release_and_broadcast();
  }

private:
  struct LockGuard {
    explicit LockGuard(const char* calling_func)
        : calling_func(calling_func), locked(false) {
      if (global_mutex.lock())
        ABORT_WITH_MESSAGE("%s failed to acquire mutex", calling_func);
      locked = true;
    }

    void release() {
      if (locked) {
        if (global_mutex.unlock())
          ABORT_WITH_MESSAGE("%s failed to release mutex", calling_func);
        locked = false;
      }
    }

    void release_and_broadcast() {
      release();
      if (global_cond.broadcast())
        ABORT_WITH_MESSAGE("%s failed to broadcast", calling_func);
    }

    ~LockGuard() { release(); }

  private:
    LockGuard() = delete;
    LockGuard(LockGuard const&) = delete;
    LockGuard& operator=(LockGuard const&) = delete;

    const char* const calling_func;
    bool locked;
  };
};

//===----------------------------------------------------------------------===//
//
//===----------------------------------------------------------------------===//

#if defined(SYS_futex)
void PlatformFutexWait(int* addr, int expect) {
  constexpr int WAIT = 0;
  syscall(SYS_futex, addr, WAIT, expect, 0);
}
void PlatformFutexWake(int* addr) {
  constexpr int WAKE = 1;
  syscall(SYS_futex, addr, WAKE, INT_MAX);
}
#else
constexpr void (*PlatformFutexWait)(int*, int) = nullptr;
constexpr void (*PlatformFutexWake)(int*) = nullptr;
#endif

template <void (*Wait)(int*, int), void (*Wake)(int*),
          uint32_t (*GetThreadIDArg)() = PlatformThreadID>
struct InitByteFutex : GuardBase<InitByteFutex<Wait, Wake, GetThreadIDArg>> {
  using BaseT = GuardBase<InitByteFutex>;
  using BaseT::BaseT;
  using BaseT::init_byte_address;

  static constexpr uint32_t (*GetThreadID)() = GetThreadIDArg;

  AcquireResult acquire_init_byte() {
    AtomicInt<uint8_t> init_byte(init_byte_address);
    while (true) {
      uint8_t last_val = 0;
      if (init_byte.compare_exchange(&last_val, PENDING_BIT, std::_AO_Acq_Rel,
                                     std::_AO_Acquire)) {
        this->store_thread_id();
        return INIT_IS_PENDING;
      } else if (last_val == COMPLETE_BIT) {
        return INIT_IS_DONE;
      } else if (last_val & PENDING_BIT) {
        this->check_thread_id_for_deadlock();
        if ((last_val & WAITING_BIT) == 0) {
          if (!init_byte.compare_exchange(&last_val, PENDING_BIT | WAITING_BIT,
                                          std::_AO_Acq_Rel, std::_AO_Release)) {
            if (last_val == COMPLETE_BIT)
              return INIT_IS_DONE;
            if (last_val == 0)
              continue;
          }
        }
        Wait(static_cast<int*>(this->base_address),
             byte_to_int_val(PENDING_BIT | WAITING_BIT));
      }
    }
  }

  void release_init_byte() {
    AtomicInt<uint8_t> init_byte(init_byte_address);
    uint8_t old = init_byte.exchange(COMPLETE_BIT, std::_AO_Acq_Rel);
    if (old & WAITING_BIT)
      Wake(static_cast<int*>(this->base_address));
  }

  void abort_init_byte() {
    AtomicInt<uint8_t> init_byte(init_byte_address);
    this->clear_thread_id();
    uint8_t old = init_byte.exchange(0, std::_AO_Acq_Rel);
    if (old & WAITING_BIT)
      Wake(static_cast<int*>(this->base_address));
  }

private:
  static int byte_to_int_val(uint8_t b) {
    int dest_val = {};
    std::memcpy(reinterpret_cast<char*>(&dest_val) + 1, &b, 1);
    return dest_val;
  }
};

//===----------------------------------------------------------------------===//
//
//===----------------------------------------------------------------------===//

template <class T>
struct GlobalStatic {
  static T instance;
};
template <class T>
_LIBCPP_SAFE_STATIC T GlobalStatic<T>::instance = {};

enum class Implementation {
  NoThreads,
  GlobalLock,
  Futex
};

constexpr Implementation CurrentImplementation =
#if defined(_LIBCXXABI_HAS_NO_THREADS)
    Implementation::NoThreads;
#elif defined(_LIBCXXABI_USE_FUTEX)
    Implementation::Futex;
#else
   Implementation::GlobalLock;
#endif

template <Implementation Impl>
struct SelectImplementation;

template <>
struct SelectImplementation<Implementation::NoThreads> {
  using type = InitByteNoThreads;
};

template <>
struct SelectImplementation<Implementation::GlobalLock> {
  using type = InitByteGlobalMutex<
      LibcppMutex, LibcppCondVar, GlobalStatic<LibcppMutex>::instance,
      GlobalStatic<LibcppCondVar>::instance, PlatformThreadID>;
};

template <>
struct SelectImplementation<Implementation::Futex> {
  using type =
      InitByteFutex<PlatformFutexWait, PlatformFutexWake, PlatformThreadID>;
};

using SelectedImplementation =
    SelectImplementation<CurrentImplementation>::type;

} // end namespace
} // end namespace __cxxabiv1

#endif // LIBCXXABI_SRC_INCLUDE_CXA_GUARD_IMPL_H
