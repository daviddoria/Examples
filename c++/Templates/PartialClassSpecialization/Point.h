#ifndef POINT_H
#define POINT_H

#include <iostream>
#include <vector>

template<typename T, typename U>
class Point
{
public:
  void function1();
  void function2();
};

template<typename T>
class Point<T, int>
{
public:
  void function1();
};

template<typename T, typename U>
void Point<T,U>::function1()
{
  std::cout << "Primary template function1" << std::endl;
}

template<typename T, typename U>
void Point<T,U>::function2()
{
  std::cout << "Primary template function2" << std::endl;
}

//////////////////
template<class T>
void Point<T, int>::function1()
{
  std::cout << "Partial specialization function1" << std::endl;
}
#endif