//===--------------------- catch_pointer_nullptr.cpp ----------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is dual licensed under the MIT and the University of Illinois Open
// Source Licenses. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include <cassert>
#include <cstdlib>
#include <iostream>

bool should_abort = false;

void do_assert(bool assert_passed, const char* msg, int line, const char* func) {
  if (assert_passed) return;
  std::cerr << __FILE__ << ":" << line << " " << func
            << ": Assertion Failed `" << msg << "'\n\n";
  should_abort = true;
}

#define my_assert(pred, msg) do_assert(pred, msg, __LINE__, __PRETTY_FUNCTION__)

#ifndef __has_feature
#define __has_feature(x) 0
#endif

#if __has_feature(cxx_nullptr)

struct Base {};
struct Derived : public Base {};

void test1()
{
    try
    {
        throw nullptr;
        assert(false);
    }
    catch (int*)
    {
    }
    catch (long*)
    {
        assert(false);
    }
}

struct A {};

void test2()
{
    try
    {
        throw nullptr;
        assert(false);
    }
    catch (A*)
    {
    }
    catch (int*)
    {
        assert(false);
    }
}

#else

void test1()
{
}

void test2()
{
}

#endif

template <class To>
bool test_conversion(To) { return true; }

template <class To>
bool test_conversion(...) { return false; }

template <class Catch>
void catch_nullptr_test() {
#if __has_feature(cxx_nullptr)
  const bool can_convert = test_conversion<Catch>(nullptr);
  try {
    throw nullptr;
    assert(false);
  } catch (Catch) {
    my_assert(can_convert, "non-convertible type incorrectly caught");
  } catch (...) {
    my_assert(can_convert, "convertible type incorrectly not caught");
  }
#endif
}

template <class Throw, class Catch>
void catch_pointer_test() {
  const bool can_convert = test_conversion<Catch>(Throw(NULL));
  try {
    throw (Throw) NULL;
    assert(false);
  } catch (Catch) {
    my_assert(can_convert, "non-convertible type incorrectly caught");
  } catch (...) {
    my_assert(!can_convert, "convertible type incorrectly not caught");
  }
}

template <class Throw, class Catch>
void catch_test() {
  catch_nullptr_test<Catch>();
  catch_pointer_test<Throw, Catch>();
}

template <class Tp>
struct TestTypes {
  typedef Tp* Type;
  typedef Tp const* CType;
  typedef Tp volatile* VType;
  typedef Tp const volatile* CVType;
};

template <class Member, class Class>
struct TestTypes<Member Class::*> {
  typedef Member (Class::*Type);
  typedef const Member (Class::*CType);
  typedef volatile Member (Class::*VType);
  typedef const volatile Member (Class::*CVType);
};


template <class Throw, class Catch, int level>
struct generate_tests {
  void operator()() {
      typedef TestTypes<Throw> Types;
      typedef typename Types::Type Type;
      typedef typename Types::CType CType;
      typedef typename Types::VType VType;
      typedef typename Types::CVType CVType;

      run_nullptr_tests();

      run_catch_tests<Type>();
      run_catch_tests<CType>();
      run_catch_tests<VType>();
      run_catch_tests<CVType>();
  }
  
  void run_nullptr_tests() {
      typedef TestTypes<Catch> Types;
      typedef typename Types::Type Type;
      typedef typename Types::CType CType;
      typedef typename Types::VType VType;
      typedef typename Types::CVType CVType;

      catch_nullptr_test<Type>();
      catch_nullptr_test<CType>();
      catch_nullptr_test<VType>();
      catch_nullptr_test<CVType>();
  }

  template <class ThrowTp>
  void run_catch_tests() {
      typedef TestTypes<Catch> Types;
      typedef typename Types::Type Type;
      typedef typename Types::CType CType;
      typedef typename Types::VType VType;
      typedef typename Types::CVType CVType;

      generate_tests<ThrowTp, Type, level-1>()();
      generate_tests<ThrowTp, CType, level-1>()();
      generate_tests<ThrowTp, VType, level-1>()();
      generate_tests<ThrowTp, CVType, level-1>()();
  }
};

template <class Throw, class Catch>
struct generate_tests<Throw, Catch, 0> {
  void operator()() {
      catch_pointer_test<Throw, Catch>();
  }
};


int main()
{
  // catch naked nullptrs
  test1();
  test2();

  //generate_tests<int, int, 1>()();
  generate_tests<int, int, 2>()();
  //generate_tests<int, int, 3>()();

  generate_tests<Base, Derived, 1>()();
  generate_tests<Base, Derived, 2>()();

  generate_tests<int A::*, int A::*, 1>()();
  generate_tests<int A::*, int A::*, 2>()();

  generate_tests<int Base::*, int Derived::*, 1>()();
  generate_tests<int Base::*, int Derived::*, 2>()();


  if (should_abort) {
    std::abort();
  }
}
