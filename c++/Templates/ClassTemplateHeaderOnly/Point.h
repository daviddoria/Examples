#ifndef POINT_H
#define POINT_H

template <typename T>
class Point
{
  T x,y,z;

public:
  double Add();
};

template <typename T>
double Point<T>::Add()
{
  return 2.0 + 4.3;
}

#endif