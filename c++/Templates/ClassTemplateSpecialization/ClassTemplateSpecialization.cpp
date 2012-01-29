#include <iostream>

#include "Point.h"

int main(int argc, char* argv[])
{
  Point<double> A;
  std::cout << A.Add( ) << std::endl;

  Point<float> B;
  std::cout << B.Add( ) << std::endl;
  return 0;
}
