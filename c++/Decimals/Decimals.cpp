#include <iostream>
#include <iomanip>

int main (int, char *[])
{

  float a = 1.23456;
  std::cout << std::fixed << std::setprecision(2) << a << std::endl;
  return 0;
}

