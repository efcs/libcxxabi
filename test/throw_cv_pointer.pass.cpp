#include <cassert>

struct S {};

int main() {
  
  try {
    throw(int**)nullptr;
  } catch (const int*const*) {
  } catch (...) {
    assert(false);
  }

  try {
    throw (int S::**)nullptr;
  } catch (const int S::*const*) {
  } catch (...) {
    assert(false);
  }
  
  try {
   throw (float S::**)nullptr;
  } catch (int S::**) {
    assert(false);
  } catch (...) {
      
  }

  return 0;
}