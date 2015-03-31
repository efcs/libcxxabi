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

template <class Throw, class Catch>
void catch_test() {
#if __has_feature(cxx_nullptr)
  {
    const bool can_convert = test_conversion<Catch>(nullptr);
    try {
      throw nullptr;
      assert(false);
    } catch (Catch) {
      assert(can_convert && "non-convertible type incorrectly caught");
    } catch (...) {
      assert(!can_convert && "convertible type incorrectly not caught");
    }
  }
#endif
  {
    const bool can_convert = test_conversion<Catch>(Throw(NULL));
    try {
      throw (Throw) NULL;
      assert(false);
    } catch (Catch) {
      assert(can_convert && "non-convertible type incorrectly caught");
    } catch (...) {
      assert(!can_convert  && "convertible type incorrectly not caught");
    }
  }
}
int main()
{
  // catch naked nullptrs
  test1();
  test2();

  catch_test<int **, int **>();                       // pointer to pointer to int

  catch_test<int **, int * const *>();                // pointer to const pointer to int
  catch_test<int **, int ** const>();                 // const pointer to pointer to int
  catch_test<int **, int * const * const>();          // const pointer to const pointer to int
  catch_test<int **, const int **>();                 // pointer to pointer to const int
  catch_test<int **, const int * const *>();          // pointer to const pointer to const int
  catch_test<int **, const int ** const>();           // const pointer to pointer to const int
  catch_test<int **, const int * const * const>();    // const pointer to const pointer to const int

  catch_test<int **, int * volatile *>();                     // pointer to volatile pointer to int
  catch_test<int **, int ** volatile>();                      // volatile pointer to pointer to int
  catch_test<int **, int * volatile * volatile>();            // volatile pointer to volatile pointer to int
  catch_test<int **, volatile int **>();                      // pointer to pointer to volatile int
  catch_test<int **, volatile int * volatile *>();            // pointer to volatile pointer to volatile int
  catch_test<int **, volatile int ** volatile>();             // volatile pointer to pointer to volatile int
  catch_test<int **, volatile int * volatile * volatile>();   // volatile pointer to volatile pointer to volatile int

  catch_test<int ***, int***>();
  catch_test<int ***, int const***>();
  catch_test<int ***, int const* const**>();
  catch_test<int ***, int const* const* const*>();

  catch_test<int ***, long***>();
  
  // Member functions
  catch_test<int A::*, int A::*>();
  catch_test<int A::*, int A::* const>();
  catch_test<int A::*, int A::* volatile>();

  catch_test<int A::*, const int A::*>();
  catch_test<int A::*, const int A::* const>();
  catch_test<int A::*, const int A::* volatile>();
  catch_test<int A::*, volatile int A::*>();
  catch_test<int A::*, volatile int A::* volatile>();
  catch_test<int A::*, volatile int A::* const>();


  catch_test<int A::**, int A::**>();
  catch_test<int A::**, int A::* const *>();
  catch_test<int A::**, int A::** const>();
  catch_test<int A::**, int A::* const* const>();

  catch_test<int A::**, const int A::* const *>();
  catch_test<int A::**, const int A::** const>();
  catch_test<int A::**, const int A::* const* const>();
  
  assert(test_conversion<int Derived::*>((int Base::*)NULL));
  catch_test<int Base::*, int Derived::*>();
  catch_test<int Derived::*, int Base::*>();

}
