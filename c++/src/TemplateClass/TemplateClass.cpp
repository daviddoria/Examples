#include "Point.h"
#include <iostream>

int main(int, char*[])
{
  Point<double> A(1.2);
  std::cout << A.getX() << std::endl;

  Point<unsigned int> B(4);
  std::cout << B.getX() << std::endl;

  return 0;
}
