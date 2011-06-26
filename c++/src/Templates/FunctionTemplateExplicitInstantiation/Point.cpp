#include "Point.h"

#include <iostream>

template<typename T>
void PrintSomething(T something)
{
  std::cout << something << std::endl;
}

template void PrintSomething<double>(double);
