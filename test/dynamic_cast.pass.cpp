//===------------------------- dynamic_cast3.cpp --------------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <cassert>

// This test explicitly tests dynamic cast with types that have inaccessible
// bases.
#if defined(__clang__)
#pragma clang diagnostic ignored "-Winaccessible-base"
#endif

namespace pr33487 {
struct Class1 {
  virtual ~Class1() {}
};
struct Shared : public virtual Class1 {};
struct Class6 : public virtual Shared {};
struct Left : public Class6 {};
struct Right : public Class6 {};
struct Main : public Left, public Right {};

void test() {
  Main m;
  Class1* c1 = &m;
  Class6* c6 = dynamic_cast<Class6*>(c1);
  assert(c6 == nullptr);
}
} // end namespace pr33487

namespace pr33439 {
struct A {
  virtual ~A() {}
};
struct B {
  virtual ~B() {}
};
struct B2 : public B {};
struct AB2 : public A, public virtual B2 {};
struct AB3 : public B2, public AB2 {};

void test() {
  AB3 ab3;
  A* a = static_cast<A*>(&ab3);
  B2* b2 = dynamic_cast<B2*>(a);
  assert(b2 == nullptr);
}
} // end namespace pr33439

namespace pr33425 {
struct Class3 {
  virtual ~Class3() {}
};
struct Class5 : protected virtual Class3 {};
struct Class6 : public virtual Class5 {};
struct Class7 : public virtual Class3 {};
struct Class9 : public Class6, public Class7 {};

void test() {
  Class9 c;
  Class3* subobj = static_cast<Class3*>(&c);
  assert(dynamic_cast<Class3*>(subobj));
  assert(dynamic_cast<Class5*>(subobj));
  assert(dynamic_cast<Class6*>(subobj)); // Fails due to PR33425
  assert(dynamic_cast<Class7*>(subobj));
  assert(dynamic_cast<Class9*>(subobj));
}
} // end namespace pr33425

int main() {
  pr33439::test();
  pr33487::test();
  pr33425::test();
}
