#include "Point.h"

template <class T>
T Point::Add(T a, T b)
{
  return a+b;
}

template int Point::Add(int, int);
template float Point::Add(float, float);