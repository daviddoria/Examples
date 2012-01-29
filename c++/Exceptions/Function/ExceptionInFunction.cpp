#include <iostream>

static void TestFunction();

int main()
{
//   try
//   {
//     TestFunction();
//   }
//   catch (...)
//   {
//     std::cerr << "Caught an exception." << std::endl;
//   }

  TestFunction();
  return 0;
}

void TestFunction()
{
  throw 0;
}
