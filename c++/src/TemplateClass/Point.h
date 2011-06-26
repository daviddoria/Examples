#ifndef POINT_H
#define POINT_H

template <typename T>
class Point
{
	T X;
public:
  Point(const T x) : X(x){}

  T getX();

};

#endif

