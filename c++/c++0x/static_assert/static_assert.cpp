#include <iostream>

int main(int argc, char* argv[])
{
  static_assert(1==2, "Wrong!"); // This will fail at compile time.
  return 0;
}
