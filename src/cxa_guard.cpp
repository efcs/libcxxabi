//===---------------------------- cxa_guard.cpp ---------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "__cxxabi_config.h"
#include "include/atomic_support.h"

#include "abort_message.h"
#include <__threading_support>

#include <stdint.h>
#include <string.h>

#define DISALLOW_COPY(T) T(T const&) = delete; T& operator=(T const&) = delete


/*
    This implementation must be careful to not call code external to this file
    which will turn around and try to call __cxa_guard_acquire reentrantly.
    For this reason, the headers of this file are as restricted as possible.
    Previous implementations of this code for __APPLE__ have used
    std::__libcpp_mutex_lock and the abort_message utility without problem. This
    implementation also uses std::__libcpp_condvar_wait which has tested
    to not be a problem.
*/

namespace __cxxabiv1
{

namespace
{

enum class ABI {
  Itanium,
  ARM
#ifdef __arm__
  Current = ARM
#else
  Current = Itanium
#endif
};

enum class Platform {
  Apple,
  POSIX,
  Other,
#ifdef __APPLE__
  Current = Apple
#elif defined(__unix__)
  Current = POSIX
#else
  Current = Other
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

enum InitializationResult {
  INIT_COMPLETE,
  INIT_NOT_COMPLETE,
};


template <ABI ABIKind>
using GuardType =
    typename std::conditional<ABIKind == ABI::ARM, uint32_t, uint64_t>::type;

template <class IntType>
struct AtomicInt {
  explicit AtomicInt(IntType *b) : b(b) {}

  IntType load(std::__libcpp_atomic_order ord) {
    return std::__libcpp_atomic_load(b, ord);
  }
  void store(IntType val, std::__libcpp_atomic_order ord) {
    std::__libcpp_atomic_store(b, val, ord);
  }
  IntType exchange(IntType new_val, std::__libcpp_atomic_order ord) {
    return std::__libcpp_atomic_exchange(b, new_val, ord);
  }
  bool compare_exchange(IntType *expected, IntType desired, std::__libcpp_atomic_order ord_success, std::__libcpp_atomic_order ord_failure) {
    return std::__libcpp_atomic_compare_exchange(b, expected, desired, ord_success, ord_failure);
  }

private:
  DISALLOW_COPY(AtomicInt);

  IntType *b;
};

template <class IntType>
struct NonAtomicInt {
  explicit NonAtomicInt(IntType *b) : b(b) {}

  IntType load(std::__libcpp_atomic_order ) {
    return *b;
  }
  void store(IntType val, std::__libcpp_atomic_order ) {
    *b = val;
  }
  IntType exchange(IntType new_val, std::__libcpp_atomic_order ) {
    IntType tmp = *b;
    *b = new_val;
    return tmp;
  }

  bool compare_exchange(IntType *expected, IntType desired, std::__libcpp_atomic_order, std::__libcpp_atomic_order) {
    if (*b == *expected) {
      *b = desired;
      return true;
    }
    *expected = *b;
    return false;
  }

 private:
  DISALLOW_COPY(NonAtomicInt);

  IntType *b;
};


using GetTIDFunc = uint32_t();

template <GetTIDFunc* = nullptr>
struct RecursiveInitDetector;

template <>
struct RecursiveInitDetector<nullptr> {
  explicit RecursiveInitDetector(uint32_t*) {}
  explicit RecursiveInitDetector(uint64_t*) {}
  bool check_for_deadlock() { return false; }
  void store_current_thread_id() {}
private:
 DISALLOW_COPY(RecursiveInitDetector);
};

template <GetTIDFunc *GetTID>
struct RecursiveInitDetector {
  explicit RecursiveInitDetector(uint64_t* g)
  : thread_id_word(static_cast<uint32_t*>(static_cast<void*>(g)) + 1) {}

  explicit RecursiveInitDetector(uint32_t*) = delete;

  bool check_for_deadlock() {
    return get_thread_id() == *thread_id_word;
  }

  void store_current_thread_id() {
    *thread_id_word = get_thread_id();
  }

 private:
  uint32_t get_thread_id() {
    if (!has_thread_id) {
      this_thread_id_cache = GetTID();
      has_thread_id = true;
    }
    return this_thread_id_cache;
  }

  bool has_thread_id = false;
  uint32_t this_thread_id_cache;
  uint32_t *thread_id_word;

 private:
  DISALLOW_COPY(RecursiveInitDetector);
};

enum class OnRelease : char { UNLOCK, UNLOCK_AND_BROADCAST };

struct GlobalMutexGuard {
  explicit GlobalMutexGuard(const char* calling_func, OnRelease on_release)
      : calling_func(calling_func), on_release(on_release) {
#ifndef _LIBCXXABI_HAS_NO_THREADS
    if (std::__libcpp_mutex_lock(&guard_mut))
      abort_message("%s failed to acquire mutex", calling_func);
#endif
  }

  ~GlobalMutexGuard() {
#ifndef _LIBCXXABI_HAS_NO_THREADS
    if (std::__libcpp_mutex_unlock(&guard_mut))
      abort_message("%s failed to release mutex", calling_func);
    if (on_release == OnRelease::UNLOCK_AND_BROADCAST) {
      if (std::__libcpp_condvar_broadcast(&guard_cv))
        abort_message("%s failed to broadcast condition variable",
                      calling_func);
    }
#endif
  }

  void wait_for_signal() {
#ifndef _LIBCXXABI_HAS_NO_THREADS
    if (std::__libcpp_condvar_wait(&guard_cv, &guard_mut))
      abort_message("%s condition variable wait failed", calling_func);
#endif
  }

private:
  GlobalMutexGuard(GlobalMutexGuard const&) = delete;
  GlobalMutexGuard& operator=(GlobalMutexGuard const&) = delete;

  const char* const calling_func;
  OnRelease on_release;

#ifndef _LIBCXXABI_HAS_NO_THREADS
  static std::__libcpp_mutex_t guard_mut;
  static std::__libcpp_condvar_t guard_cv;
#endif
};

#ifndef _LIBCXXABI_HAS_NO_THREADS
std::__libcpp_mutex_t GlobalMutexGuard::guard_mut = _LIBCPP_MUTEX_INITIALIZER;
std::__libcpp_condvar_t GlobalMutexGuard::guard_cv =
    _LIBCPP_CONDVAR_INITIALIZER;
#endif

struct GuardObject;


static constexpr uint8_t INIT_COMPLETE_BIT = 1;
static constexpr uint8_t INIT_PENDING_BIT = 2;
static constexpr uint8_t WAITING_BIT = 4;

template <Implementation Impl>
class InitFlagsByte;

template <Implementation Impl>
struct InitFlagsByte {
  bool is_initialization_complete();
  bool is_initializ
private:
   NonAtomicInt<uint8_t> byte;
};

#if 0
/// GuardValue - An abstraction for accessing the various fields and bits of
///   the guard object.
struct GuardValue {
private:
  explicit GuardValue(guard_type v) : value(v) {}
  friend struct GuardObject;

public:
  /// Functions returning the values used to represent the uninitialized,
  /// initialized, and initialization pending states.
  static GuardValue ZERO();
  static GuardValue INIT_COMPLETE();
  static GuardValue INIT_PENDING();

  /// Returns true if the guard value represents that the initialization is
  /// complete.
  bool is_initialization_complete() const;

  /// Returns true if the guard value represents that the initialization is
  /// currently pending.
  bool is_initialization_pending() const;

  /// Returns the lock value for the current guard value.
  lock_type get_lock_value() const;

private:
  // Returns a guard object corresponding to the specified lock value.
  static guard_type guard_value_from_lock(lock_type l);

  // Returns the lock value represented by the specified guard object.
  static lock_type lock_value_from_guard(guard_type g);

private:
  guard_type value;
};
#endif

template <Implementation Impl, ABI ABIKind, Platform Plat>
struct ImplementationTypes {
  using guard_type = GuardType<ABIKind>;
  using guard_byte_t = typename std::conditional<
      Impl == Implementation::NoThreads,
      NonAtomicByte,
      AtomicByte>::type;
  using init_byte_t = typename std::conditional<
      Impl != Implementation::Futex,
      NonAtomicByte,
      AtomicByte>::type;
  using recursive_init_detector_t = typename std::conditional<
      ABIKind == ABI::Itanium &&
      Plat != Platform::Other &&
      Impl != Implementation::NoThreads,
      RecursiveInitDetector<nullptr>,
      RecursiveInitDetector<nullptr>
    >::type;
  
  
};

/// GuardObject - Manages correctly reading and writing to the guard object.
template <class GuardType, class InitByteT, class RecInitDetector>
struct GuardObject {
  explicit GuardObject(GuardType *g) : guard(g),
    guard_byte(static_cast<uint8_t*>(static_cast<void*>(g))),
    init_byte(static_cast<uint8_t*>(static_cast<void*>(g)) + 1),
    rec_init(g)
     {}

  // Read the current value of the guard object.
  // TODO: Make this read atomic.
  uint8_t read_init_byte() const;

  // Write the specified value to the guard object.
  // TODO: Make this atomic
  void write_init_byte(uint8_t new_val);

  bool acquire_guard() { return guard_byte.load(std::_AO_Acquire); }
  void set_guard_and_release() { return guard_byte.store(1, std::_AO_Release); }

  GuardType *guard;
  AtomicByte guard_byte;
  InitByteT init_byte;
  RecInitDetector rec_init;

private:
  GuardObject(const GuardObject&) = delete;
  GuardObject& operator=(const GuardObject&) = delete;

};

template <class GuardObjectT>
int cxa_guard_acquire(typename GuardObjectT::GuardType *g) {
  GuardObjectT guard(g);
  if (guard.guard_byte.load(std::_AO_Acquire) == INIT_COMPLETE_BIT)
    return INIT_COMPLETE;
    
  
  return INIT_NOT_COMPLETE;
}

template <class GuardObjectT>
void cxa_guard_release(typename GuardObjectT::GuardType *g) {
  GuardObjectT guard(g);
  guard.init_byte.store(INIT_COMPLETE_BIT, std::_AO_Release);
  guard.guard_byte.store(INIT_COMPLETE_BIT, std::_AO_Release);
}


template <class GuardObjectT>
void cxa_guard_abort(typename GuardObjectT::GuardType *g) {
  GuardObjectT guard(g);
  guard.init_byte.store(0, std::_AO_Release);
}





}  // unnamed namespace

extern "C"
{

_LIBCXXABI_FUNC_VIS int __cxa_guard_acquire(guard_type* raw_guard_object) {
  GlobalMutexGuard gmutex("__cxa_guard_acquire", OnRelease::UNLOCK);
  GuardObject guard(raw_guard_object);
  
  GuardValue current_value = guard.read();

  if (current_value.is_initialization_complete())
    return INIT_COMPLETE;

  const GuardValue LOCK_ID = GuardValue::INIT_PENDING();
#ifdef LIBCXXABI_HAS_DEADLOCK_DETECTION
   if (current_value.is_initialization_pending() &&
       current_value.get_lock_value() == LOCK_ID.get_lock_value()) {
    abort_message("__cxa_guard_acquire detected deadlock");
  }
#endif
  while (current_value.is_initialization_pending()) {
      gmutex.wait_for_signal();
      current_value = guard.read();
  }
  if (current_value.is_initialization_complete())
    return INIT_COMPLETE;

  guard.write(LOCK_ID);
  return INIT_NOT_COMPLETE;
}

_LIBCXXABI_FUNC_VIS void __cxa_guard_release(guard_type *raw_guard_object) {
  GlobalMutexGuard gmutex("__cxa_guard_release",
                          OnRelease::UNLOCK_AND_BROADCAST);
  GuardObject guard(raw_guard_object);
  guard.write(GuardValue::ZERO());
  guard.write(GuardValue::INIT_COMPLETE());
}

_LIBCXXABI_FUNC_VIS void __cxa_guard_abort(guard_type *raw_guard_object) {
  GlobalMutexGuard gmutex("__cxa_guard_abort", OnRelease::UNLOCK_AND_BROADCAST);
  GuardObject guard(raw_guard_object);
  guard.write(GuardValue::ZERO());
}
}  // extern "C"

//===----------------------------------------------------------------------===//
//                        GuardObject Definitions
//===----------------------------------------------------------------------===//

GuardValue GuardObject::read() const {
  // FIXME: Make this atomic
  guard_type val = *guard;
  return GuardValue(val);
}

void GuardObject::write(GuardValue new_val) {
  // FIXME: make this atomic
  *guard = new_val.value;
}

//===----------------------------------------------------------------------===//
//                        GuardValue Definitions
//===----------------------------------------------------------------------===//

GuardValue GuardValue::ZERO() { return GuardValue(0); }

GuardValue GuardValue::INIT_COMPLETE() {
  guard_type value = {0};
#if defined(_LIBCXXABI_GUARD_ABI_ARM)
  value |= 1;
#else
  char* init_bit = (char*)&value;
  *init_bit = 1;
#endif
  return GuardValue(value);
}

GuardValue GuardValue::INIT_PENDING() {
  return GuardValue(guard_value_from_lock(LOCK_ID_FOR_THREAD()));
}

bool GuardValue::is_initialization_complete() const {
#if defined(_LIBCXXABI_GUARD_ABI_ARM)
  return value & 1;
#else
  const char* init_bit = (const char*)&value;
  return *init_bit;
#endif
}

bool GuardValue::is_initialization_pending() const {
  return lock_value_from_guard(value) != 0;
}

lock_type GuardValue::get_lock_value() const {
  return lock_value_from_guard(value);
}

// Create a guard object with the lock set to the specified value.
guard_type GuardValue::guard_value_from_lock(lock_type l) {
#if defined(__APPLE__) && !defined(_LIBCXXABI_GUARD_ABI_ARM)
#if __LITTLE_ENDIAN__
  return static_cast<guard_type>(l) << 32;
#else
  return static_cast<guard_type>(l);
#endif
#else  // defined(__APPLE__) && !defined(_LIBCXXABI_GUARD_ABI_ARM)
  guard_type f = {0};
  memcpy(static_cast<char*>(static_cast<void*>(&f)) + 1, &l, sizeof(lock_type));
  return f;
#endif // defined(__APPLE__) && !defined(_LIBCXXABI_GUARD_ABI_ARM)
}

lock_type GuardValue::lock_value_from_guard(guard_type g) {
#if defined(__APPLE__) && !defined(_LIBCXXABI_GUARD_ABI_ARM)
#if __LITTLE_ENDIAN__
  return static_cast<lock_type>(g >> 32);
#else
  return static_cast<lock_type>(g);
#endif
#else  // defined(__APPLE__) && !defined(_LIBCXXABI_GUARD_ABI_ARM)
  uint8_t guard_bytes[sizeof(guard_type)];
  memcpy(&guard_bytes, &g, sizeof(guard_type));
  return guard_bytes[1] != 0;
#endif // defined(__APPLE__) && !defined(_LIBCXXABI_GUARD_ABI_ARM)
}

}  // __cxxabiv1
