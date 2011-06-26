#ifndef POINT_H
#define POINT_H

class Point
{
  double x,y,z;

public:
  template <typename T>
  double Add();
};

template <typename T>
double Point::Add()
{
  return 2.0 + 4.3;
}

#endif