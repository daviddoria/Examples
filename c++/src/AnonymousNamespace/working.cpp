#include <iostream>

namespace
{
  void Test();
}

int main (int argc, char *argv[])
{

  Test();
  return 0;
}

namespace
{
  void Test()
  {
  std::cout << "Hi" << std::endl;
  }
}