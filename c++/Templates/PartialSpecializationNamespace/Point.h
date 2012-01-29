#ifndef POINT_H
#define POINT_H

#include <iostream>
#include <vector>

namespace TestNamespace
{
  
template<class T, class U, int I> struct X
  {
    void f();
  };

template<class T, int I> struct X<T, T*, I>
  {
    void f();
  };

}

#include "Point.txx"

#endif

