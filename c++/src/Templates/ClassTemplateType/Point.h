#ifndef POINT_H
#define POINT_H

template <typename T>
class Point
{
public:
  typedef T PointType;
  T x;
  T GetX(){return x;}
};


#endif