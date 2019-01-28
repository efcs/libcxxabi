//===----------------------------------------------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// REQUIRES: libcxxabi-potentially-blocking-region-hooks

// extern "C" void __libcpp_potentially_blocking_region_begin(void);
// extern "C" void __libcpp_potentially_blocking_region_begin(void);

// Test that potentially blocking region hooks can be overridden by the
// user and that they are called by `__cxa_guard_acquire`.

#include <cassert>

int begin_called = 0;
int end_called = 0;

extern "C" void __libcppabi_potentially_blocking_region_begin() {
  ++begin_called;
}

extern "C" void __libcppabi_potentially_blocking_region_end() {
  ++end_called;
}

extern int init_val;

template <int>
void test_fn() {
  static int dummy = init_val;
  ((void)dummy);
}

int init_val = 0;

int main() {
  assert(begin_called == end_called);
  begin_called = end_called = 0;
  test_fn<1>();
  assert(begin_called == 1);
  assert(end_called == 1);
}
