#include <iostream>

#include "Point.h"

int main()
{
  Point<int, double> a;
  Point<int, int> b;

  a.function1();
  b.function1();

  a.function2();
  b.function2();
}