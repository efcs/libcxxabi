#ifndef LIBCXXABI_SRC_INCLUDE_CXA_GUARD_IMPL_H
#define LIBCXXABI_SRC_INCLUDE_CXA_GUARD_IMPL_H

#include "__cxxabi_config.h"
#include "atomic_support.h"
#include <unistd.h>
#include <sys/syscall.h>
#include <sys/types.h>

#ifndef ABORT_WITH_MESSAGE
#include "../abort_message.h"
#define ABORT_WITH_MESSAGE(...) ::abort_message(__VA_ARGS__)
#endif

#include <__threading_support>

#define DISALLOW_COPY(T)                                                       \
  T(T const&) = delete;                                                        \
  T& operator=(T const&) = delete

#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#endif

namespace __cxxabiv1 {

namespace {

enum class ABI {
  Itanium,
  ARM,
#ifdef __arm__
  Current = ARM
#else
  Current = Itanium
#endif
};

enum class Implementation {
  NoThreads,
  GlobalLock,
  Futex,
#if defined(_LIBCXXABI_HAS_NO_THREADS)
  Current = NoThreads
#elif defined(_LIBCXXABI_USE_FUTEX)
  Current = Futex
#else
  Current = GlobalLock
#endif
};

enum class AcquireResult {
  INIT_IS_DONE,
  INIT_IS_PENDING,
};
constexpr AcquireResult INIT_IS_DONE = AcquireResult::INIT_IS_DONE;
constexpr AcquireResult INIT_IS_PENDING = AcquireResult::INIT_IS_PENDING;

#if defined(__APPLE__) && defined(_LIBCPP_HAS_THREAD_API_PTHREAD)
uint32_t PlatformThreadID() {
  return pthread_mach_thread_np(std::__libcpp_thread_get_current_id());
}
#elif defined(SYS_gettid) && defined(_LIBCPP_HAS_THREAD_API_PTHREAD)
uint32_t PlatformThreadID() {
  static_assert(sizeof(pid_t) == sizeof(uint32_t), "");
  return static_cast<uint32_t>(static_cast<pid_t>(syscall(SYS_gettid)));
}
#else
constexpr uint32_t (*PlatformThreadID)() = nullptr;
#endif

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

struct LibcppMutex;
struct LibcppCondVar;

#ifndef _LIBCXXABI_HAS_NO_THREADS
struct LibcppMutex {
  LibcppMutex() = default;
  bool lock() { return std::__libcpp_mutex_lock(&mutex); }
  bool unlock() { return std::__libcpp_mutex_unlock(&mutex); }

private:
  friend struct LibcppCondVar;
  DISALLOW_COPY(LibcppMutex);
  std::__libcpp_mutex_t mutex = _LIBCPP_MUTEX_INITIALIZER;
};

struct LibcppCondVar {
  LibcppCondVar() = default;
  bool wait(LibcppMutex& mut) {
    return std::__libcpp_condvar_wait(&cond, &mut.mutex);
  }
  bool broadcast() { return std::__libcpp_condvar_broadcast(&cond); }

private:
  DISALLOW_COPY(LibcppCondVar);
  std::__libcpp_condvar_t cond = _LIBCPP_CONDVAR_INITIALIZER;
};
#endif

static constexpr uint8_t INIT_COMPLETE_BIT = 1;
static constexpr uint8_t INIT_PENDING_BIT = 2;
static constexpr uint8_t WAITING_BIT = 4;

template <class Derived>
struct GuardImplBase {
  GuardImplBase() = delete;

  explicit GuardImplBase(uint32_t* g)
      : base_address(g), guard_byte_address(reinterpret_cast<uint8_t*>(g)),
        init_byte_address(reinterpret_cast<uint8_t*>(g) + 1),
        thread_id_address(nullptr) {}
  explicit GuardImplBase(uint64_t* g)
      : base_address(g), guard_byte_address(reinterpret_cast<uint8_t*>(g)),
        init_byte_address(reinterpret_cast<uint8_t*>(g) + 1),
        thread_id_address(reinterpret_cast<uint32_t*>(g) + 1) {}

  AcquireResult acquire() {
    AtomicInt<uint8_t> guard_byte(guard_byte_address);
    if (guard_byte.load(std::_AO_Acquire) == INIT_COMPLETE_BIT)
      return INIT_IS_DONE;
    return derived()->acquire_init_byte();
  }

  void release() {
    AtomicInt<uint8_t> guard_byte(guard_byte_address);
    guard_byte.store(INIT_COMPLETE_BIT, std::_AO_Release);
    derived()->release_init_byte();
  }

  void abort() { derived()->abort_init_byte(); }

  Derived* derived() { return static_cast<Derived*>(this); }

  void* const base_address;
  uint8_t* const guard_byte_address;
  uint8_t* const init_byte_address;
  uint32_t* const thread_id_address;

protected:
  void check_thread_id_for_deadlock() {
    if (!check_thread_id)
      return;
    AtomicInt<uint32_t> thread_id(thread_id_address);
    if (thread_id.load(std::_AO_Relaxed) == current_thread_id()) {
      ABORT_WITH_MESSAGE("__cxa_guard_acquire detected deadlock");
    }
  }

  void clear_thread_id() {
    if (!check_thread_id)
      return;
    AtomicInt<uint32_t> thread_id(thread_id_address);
    thread_id.store(0, std::_AO_Relaxed);
  }

  void store_thread_id() {
    if (!check_thread_id)
      return;
    AtomicInt<uint32_t> thread_id(thread_id_address);
    thread_id.store(current_thread_id(), std::_AO_Relaxed);
  }

private:
  uint32_t current_thread_id() {
    if (check_thread_id && !has_current_thread_id) {
      current_thread_id_cache = Derived::GetThreadID();
      has_current_thread_id = true;
    }
    return current_thread_id_cache;
  }

  const bool check_thread_id = thread_id_address && Derived::GetThreadID;
  bool has_current_thread_id = false;
  uint32_t current_thread_id_cache = 0;
  DISALLOW_COPY(GuardImplBase);
};

struct NoThreadsImpl : GuardImplBase<NoThreadsImpl> {
  using GuardImplBase::GuardImplBase;

  static constexpr uint32_t (*GetThreadID)() = nullptr;

  AcquireResult acquire_init_byte() {
    if (*init_byte_address == INIT_COMPLETE_BIT)
      return INIT_IS_DONE;
    *init_byte_address = INIT_PENDING_BIT;
    return INIT_IS_PENDING;
  }

  void release_init_byte() { *init_byte_address = INIT_COMPLETE_BIT; }
  void abort_init_byte() { *init_byte_address = 0; }
};

template <class Mutex, Mutex& global_mutex, class CondVar, CondVar& global_cond,
          uint32_t (*GetThreadIDArg)() = PlatformThreadID>
struct GlobalMutexImpl
    : GuardImplBase<GlobalMutexImpl<Mutex, global_mutex, CondVar, global_cond,
                                    GetThreadIDArg>> {
  using BaseT = GuardImplBase<GlobalMutexImpl>;
  using BaseT::BaseT;

  using BaseT::init_byte_address;

  static constexpr uint32_t (*GetThreadID)() = GetThreadIDArg;

  AcquireResult acquire_init_byte() {
    LockGuard g("__cxa_guard_acquire");
    if (*init_byte_address & INIT_PENDING_BIT) {
      this->check_thread_id_for_deadlock();
    }
    while (*init_byte_address & INIT_PENDING_BIT) {
      *init_byte_address |= WAITING_BIT;
      global_cond.wait(global_mutex);
    }
    if (*init_byte_address)
      return INIT_IS_DONE;
    this->store_thread_id();
    *init_byte_address = INIT_PENDING_BIT;
    return INIT_IS_PENDING;
  }

  void release_init_byte() {
    LockGuard g("__cxa_guard_release");
    bool has_waiting = *init_byte_address & WAITING_BIT;
    *init_byte_address = INIT_COMPLETE_BIT;
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
    DISALLOW_COPY(LockGuard);
    const char* const calling_func;
    bool locked;
  };
};

template <void (*Wait)(int*, int), void (*Wake)(int*),
          uint32_t (*GetThreadIDArg)() = PlatformThreadID>
struct FutexImpl : GuardImplBase<FutexImpl<Wait, Wake, GetThreadIDArg>> {
  using BaseT = GuardImplBase<FutexImpl>;
  using BaseT::BaseT;
  using BaseT::init_byte_address;

  static constexpr uint32_t (*GetThreadID)() = GetThreadIDArg;

  AcquireResult acquire_init_byte() {
    AtomicInt<uint8_t> init_byte(init_byte_address);
    while (true) {
      uint8_t last_val = 0;
      if (init_byte.compare_exchange(&last_val, INIT_PENDING_BIT,
                                     std::_AO_Acq_Rel, std::_AO_Acquire)) {
        this->store_thread_id();
        return INIT_IS_PENDING;
      } else if (last_val == INIT_COMPLETE_BIT) {
        return INIT_IS_DONE;
      } else if (last_val & INIT_PENDING_BIT) {
        this->check_thread_id_for_deadlock();
        if ((last_val & WAITING_BIT) == 0) {
          if (!init_byte.compare_exchange(&last_val,
                                          INIT_PENDING_BIT | WAITING_BIT,
                                          std::_AO_Acq_Rel, std::_AO_Release)) {
            if (last_val == INIT_COMPLETE_BIT)
              return INIT_IS_DONE;
            if (last_val == 0)
              continue;
          }
        }
        Wait(static_cast<int*>(this->base_address),
             byte_to_int_val(INIT_PENDING_BIT | WAITING_BIT));
      }
    }
  }

  void release_init_byte() {
    AtomicInt<uint8_t> init_byte(init_byte_address);
    uint8_t old = init_byte.exchange(INIT_COMPLETE_BIT, std::_AO_Acq_Rel);
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

template <Implementation Impl>
struct ChooseImplementation;

template <>
struct ChooseImplementation<Implementation::NoThreads> {
  using type = NoThreadsImpl;
};

template <class T>
struct GlobalStatic {
  static T instance;
};
template <class T>
_LIBCPP_SAFE_STATIC T GlobalStatic<T>::instance = {};

template <>
struct ChooseImplementation<Implementation::GlobalLock> {
  using type =
      GlobalMutexImpl<LibcppMutex, GlobalStatic<LibcppMutex>::instance,
                      LibcppCondVar, GlobalStatic<LibcppCondVar>::instance,
                      PlatformThreadID>;
};

template <>
struct ChooseImplementation<Implementation::Futex> {
  using type =
      FutexImpl<PlatformFutexWait, PlatformFutexWake, PlatformThreadID>;
};

using CurrentImplementation =
    ChooseImplementation<Implementation::Current>::type;

} // end namespace
} // end namespace __cxxabiv1

#endif // LIBCXXABI_SRC_INCLUDE_CXA_GUARD_IMPL_H
