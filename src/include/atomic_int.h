#ifndef LIBCXXABI_SRC_INCLUDE_ATOMIC_INT_H
#define LIBCXXABI_SRC_INCLUDE_ATOMIC_INT_H


#include "atomic_support.h"

namespace {

using MemoryOrder = std::__libcpp_atomic_order;

template <class IntType>
struct AtomicInt {
  explicit AtomicInt(IntType *b) : b(b) {}

  IntType load(MemoryOrder ord) {
    return std::__libcpp_atomic_load(b, ord);
  }
  void store(IntType val, MemoryOrder ord) {
    std::__libcpp_atomic_store(b, val, ord);
  }
  IntType exchange(IntType new_val, MemoryOrder ord) {
    return std::__libcpp_atomic_exchange(b, new_val, ord);
  }
  bool compare_exchange(IntType *expected, IntType desired, MemoryOrder ord_success, MemoryOrder ord_failure) {
    return std::__libcpp_atomic_compare_exchange(b, expected, desired, ord_success, ord_failure);
  }

 private:
  AtomicInt(AtomicInt const&) = delete;
  AtomicInt& operator=(AtomicInt const&) = delete;

  IntType *b;
};

template <class IntType>
struct NonAtomicInt {
  explicit NonAtomicInt(IntType *b) : b(b) {}

  IntType load(MemoryOrder ) {
    return *b;
  }
  void store(IntType val, MemoryOrder ) {
    *b = val;
  }
  IntType exchange(IntType new_val, MemoryOrder ) {
    IntType tmp = *b;
    *b = new_val;
    return tmp;
  }

  bool compare_exchange(IntType *expected, IntType desired, MemoryOrder, MemoryOrder) {
    if (*b == *expected) {
      *b = desired;
      return true;
    }
    *expected = *b;
    return false;
  }

private:
  NonAtomicInt(NonAtomicInt const&) = delete;
  NonAtomicInt& operator=(NonAtomicInt const&) = delete;

  IntType *b;
};

} // end namespace

#endif // LIBCXXABI_SRC_INCLUDE_ATOMIC_INT_H
