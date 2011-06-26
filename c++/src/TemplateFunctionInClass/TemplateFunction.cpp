#include <iostream>

#include "Point.h"

int main(int, char*[])
{
  double a = 1.2;
  PrintSomething(a);

  // Sometimes have to do this:
  PrintSomething<double>(a);

  return 0;
}
