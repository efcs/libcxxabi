struct S {};

int main() {
  try {
    throw(int**)nullptr;
  } catch (const int*const*) {
  } catch (...) {
    return 42;
  }

  try {
    throw (int S::**)nullptr;
  } catch (const int S::*const*) {
  } catch (...) {
    return 42;
  }

  return 0;
}