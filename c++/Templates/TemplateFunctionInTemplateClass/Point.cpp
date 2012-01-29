#include "Point.h"

template<typename T>
template<typename S>
void Point<T>::PrintSomething(S something)
{
  std::cout << something << std::endl;
}

template void Point<double>::PrintSomething<int>(int);
template void Point<double>::PrintSomething<double>(double);
