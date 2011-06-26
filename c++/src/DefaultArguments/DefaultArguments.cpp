#include <iostream>
#include <limits>

void Test(int a = 5);

int main(int argc, char *argv[])
{
  Test();
  return 0;
}

void Test(int a)
{
  std::cout << a << std::endl;
}