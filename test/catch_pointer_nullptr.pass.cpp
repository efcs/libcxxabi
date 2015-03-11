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

template <class T>
void test_conversion(T) {}

template <typename T, typename C>
void catch_test() {
#if __has_feature(cxx_nullptr)
   try { throw (T) nullptr; assert(false); }
   catch (C) {}
   catch (...) { assert(false); }
#endif

   test_conversion<C>((T)0);

   try { throw (T) NULL; assert(false); }
   catch (C) {}
   catch (...) { assert(false); }
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
   //catch_test<int **, const int **>();                 // pointer to pointer to const int
   catch_test<int **, const int * const *>();          // pointer to const pointer to const int
   //catch_test<int **, const int ** const>();           // const pointer to pointer to const int
   catch_test<int **, const int * const * const>();    // const pointer to const pointer to const int

   catch_test<int **, int * volatile *>();                     // pointer to volatile pointer to int
   catch_test<int **, int ** volatile>();                      // volatile pointer to pointer to int
   catch_test<int **, int * volatile * volatile>();            // volatile pointer to volatile pointer to int
   //catch_test<int **, volatile int **>();                      // pointer to pointer to volatile int
   //catch_test<int **, volatile int * volatile *>();            // pointer to volatile pointer to volatile int
   //catch_test<int **, volatile int ** volatile>();             // volatile pointer to pointer to volatile int
   //catch_test<int **, volatile int * volatile * volatile>();   // volatile pointer to volatile pointer to volatile int


// Member functions
   catch_test<int A::*, int A::*>();
   //catch_test<int A::*, int A::* const>();
   //catch_test<int A::*, int A::* volatile>();
/*
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
    //catch_test<int A::**, const int A::** const>();
    catch_test<int A::**, const int A::* const* const>();
    */
}
