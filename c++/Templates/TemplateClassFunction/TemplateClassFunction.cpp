#include <iostream>

#include "Point.h"

int main(int argc, char* argv[])
{
	Point P;
  float a = 1.0;
  float b = 2.0;
	std::cout << P.Add(a,b) << std::endl;

  int c = 1.0;
  int d = 2.0;
  std::cout << P.Add(c,d) << std::endl;
	return 0;
}
