#include <iostream>

#include "Point.h"

int main(int, char*[])
{
  PrintSomething(1.0);

  // Sometimes have to do this:
  PrintSomething<double>(1.0);

  return 0;
}
