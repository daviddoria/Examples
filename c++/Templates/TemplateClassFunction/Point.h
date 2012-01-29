
#ifndef POINT_H
#define POINT_H

/*
// works
class Point
{
	public:

  template <class T>
  T Add(T a, T b)
  {
    return a+b;
  }
};
*/

/*
// works
class Point
{
  public:

  template <class T>
  T Add(T a, T b);
};

template <class T>
T Point::Add(T a, T b)
{
  return a+b;
}
*/

// This works if you use Point.cpp
class Point
{
  public:

  template <class T>
  T Add(T a, T b);


};



#endif
