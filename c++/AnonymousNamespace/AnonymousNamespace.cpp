#include <iostream>

namespace TestName
{
  void Test();
}

int main (int argc, char *argv[])
{

  TestName::Test();
  return 0;
}


namespace TestName
{
  void Test()
  {
  std::cout << "Hi" << std::endl;
  }
}