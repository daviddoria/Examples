#include <iostream>

#include "Point.h"

int main(int, char*[])
{
  Point<double> DoublePoint;
  int a = 2;
  DoublePoint.PrintSomething(a);

  double b = 2.1;
  DoublePoint.PrintSomething(b);
  // Sometimes have to do this:
  //PrintSomething<double>(a);

  return 0;
}
