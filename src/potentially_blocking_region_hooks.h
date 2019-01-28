//===----------------------------------------------------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LIBCXXABI_SRC_POTENTIALLY_BLOCKING_REGION_HOOKS_H
#define LIBCXXABI_SRC_POTENTIALLY_BLOCKING_REGION_HOOKS_H

#include "__cxxabi_config.h"

#ifdef LIBCXXABI_ENABLE_POTENTIALLY_BLOCKING_REGION_HOOKS
extern "C" {
  _LIBCXXABI_FUNC_VIS void __libcppabi_potentially_blocking_region_begin(void);
  _LIBCXXABI_FUNC_VIS void __libcppabi_potentially_blocking_region_end(void);
} // extern "C"
#else
namespace {
  void __libcppabi_potentially_blocking_region_begin() {}
  void __libcppabi_potentially_blocking_region_end() {}
} // end namespace
#endif // LIBCXXABI_ENABLE_POTENTIALLY_BLOCKING_REGION_HOOKS

namespace {
struct potentially_blocking_region {
  potentially_blocking_region() {
    __libcppabi_potentially_blocking_region_begin();
  }
  ~potentially_blocking_region() {
    __libcppabi_potentially_blocking_region_end();
  }
private:
  potentially_blocking_region(potentially_blocking_region const&) = delete;
  potentially_blocking_region& operator=(potentially_blocking_region const&) = delete;
};

} // end namespace
#endif // LIBCXXABI_SRC_POTENTIALLY_BLOCKING_REGION_HOOKS_H
