#include <iostream>

#include "Point.h"

int main(int argc, char* argv[])
{
  Point A;

  std::cout << A.Add<double>() << std::endl;
  return 0;
}
