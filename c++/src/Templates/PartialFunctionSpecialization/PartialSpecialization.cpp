#include <iostream>

#include "Point.h"

int main()
{
   X<int, int, 10> a;
   X<int, int*, 5> b;

//   X<int, int*, 10> f;
   a.f(); b.f();
}