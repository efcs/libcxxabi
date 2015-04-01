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

void do_assert(bool assert_passed, const char* msg, int line, const char* func) {
  if (assert_passed) return;
  std::cerr << __FILE__ << ":" << line << " " << func
            << ": Assertion Failed `" << msg << "'\n\n";
  std::abort();
}

#define my_assert(pred, msg) do_assert(pred, msg, __LINE__, __PRETTY_FUNCTION__)

#ifndef __has_feature
#define __has_feature(x) 0
#endif

struct A {};
struct Base {};
struct Derived : public Base {};

#if __has_feature(cxx_nullptr)

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


template <class Pointer>
struct CreatePointer {
  Pointer operator()() const {
      return (Pointer)0;
  }
};


template <class Tp>
struct CreatePointer<Tp*> {
  Tp* operator()() const {
      return (Tp*)42;
  }
};

std::size_t test_count = 0;

template <class Throw, class Catch>
void catch_pointer_test() {
  ++test_count;
  Throw throw_ptr = CreatePointer<Throw>()();
  const bool can_convert = test_conversion<Catch>(throw_ptr);
  try {
    throw throw_ptr;
    assert(false);
  } catch (Catch catch_ptr) {
    Catch catch2 = CreatePointer<Catch>()();
    my_assert(can_convert, "non-convertible type incorrectly caught");
    my_assert(catch_ptr == catch2,
              "Thrown pointer does not match caught ptr");
  } catch (...) {
    my_assert(!can_convert, "convertible type incorrectly not caught");
  }
}

template <class Throw, class Catch>
void catch_test() {
  catch_nullptr_test<Catch>();
  catch_pointer_test<Throw, Catch>();
}

template <class Tp, bool First = false>
struct TestTypes {
  typedef Tp* Type;
  typedef Tp const* CType;
  typedef Tp volatile* VType;
  typedef Tp const volatile* CVType;
};

template <class Member, class Class>
struct TestTypes<Member Class::*, true> {
  typedef Member (Class::*Type);
  typedef const Member (Class::*CType);
  typedef volatile Member (Class::*VType);
  typedef const volatile Member (Class::*CVType);
};

template <class Throw, class Catch, int level, bool first = false>
struct generate_tests_imp {
  typedef TestTypes<Throw, first> ThrowTypes;
  typedef TestTypes<Catch, first> CatchTypes;
  void operator()() {
      typedef typename ThrowTypes::Type Type;
      typedef typename ThrowTypes::CType CType;
      typedef typename ThrowTypes::VType VType;
      typedef typename ThrowTypes::CVType CVType;

      run_nullptr_tests();

      run_catch_tests<Type>();
      run_catch_tests<CType>();
      run_catch_tests<VType>();
      run_catch_tests<CVType>();
  }
  
  void run_nullptr_tests() {
      typedef typename CatchTypes::Type Type;
      typedef typename CatchTypes::CType CType;
      typedef typename CatchTypes::VType VType;
      typedef typename CatchTypes::CVType CVType;

      catch_nullptr_test<Type>();
      catch_nullptr_test<CType>();
      catch_nullptr_test<VType>();
      catch_nullptr_test<CVType>();
  }

  template <class ThrowTp>
  void run_catch_tests() {
      typedef typename CatchTypes::Type Type;
      typedef typename CatchTypes::CType CType;
      typedef typename CatchTypes::VType VType;
      typedef typename CatchTypes::CVType CVType;

      catch_pointer_test<ThrowTp, Type>();
      catch_pointer_test<ThrowTp, CType>();
      catch_pointer_test<ThrowTp, VType>();
      catch_pointer_test<ThrowTp, CVType>();

      generate_tests_imp<ThrowTp, Type, level-1>()();
      generate_tests_imp<ThrowTp, CType, level-1>()();
      generate_tests_imp<ThrowTp, VType, level-1>()();
      generate_tests_imp<ThrowTp, CVType, level-1>()();
  }
};


template <class Throw, class Catch, bool first>
struct generate_tests_imp<Throw, Catch, 0, first> {
  void operator()() {
      catch_pointer_test<Throw, Catch>();
  }
};

template <class Throw, class Catch, int level>
struct generate_tests : generate_tests_imp<Throw, Catch, level, true> {};

int main()
{
  // catch naked nullptrs
  test1();
  test2();

  generate_tests<int, int, 3>()();
  generate_tests<Base, Derived, 2>()();
  generate_tests<Derived, Base, 2>()();
  generate_tests<int, void, 2>()();
  generate_tests<void, int, 2>()();

  generate_tests<int A::*, int A::*, 3>()();
  generate_tests<int A::*, void, 2>()();
  generate_tests<int Base::*, int Derived::*, 2>()();
  generate_tests<int Derived::*, int Base::*, 2>()();
}
