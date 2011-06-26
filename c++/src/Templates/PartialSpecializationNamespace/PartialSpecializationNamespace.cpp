#include <iostream>

#include "Point.h"

int main()
{
   TestNamespace::X<int, int, 10> a;
   TestNamespace::X<int, int*, 5> b;

//   X<int, int*, 10> f;
   a.f(); b.f();
}