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

template<typename T, typename U>
void Point<T, U>::function1() { };

template<typename T, typename U>
void Point<T, U>::function2() { };

template<typename T>
void Point<T, int>::function1() { };

#endif