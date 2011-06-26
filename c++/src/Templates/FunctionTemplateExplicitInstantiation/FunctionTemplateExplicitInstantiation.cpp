#include <iostream>

#include "Point.h"

int main(int, char*[])
{
  PrintSomething(1.2);

  // Sometimes have to do this:
  PrintSomething<double>(1.2);

  return 0;
}
