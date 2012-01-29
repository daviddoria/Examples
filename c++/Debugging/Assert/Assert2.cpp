#include <iostream>

//#undef NDEBUG // If this is placed before including cassert, the asserts will always be hit, no matter if the program is built in debug or release mode
#include <cassert>

int main()
{
  assert("test" && (1 == 2));
  return 0;
}
