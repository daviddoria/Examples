#ifndef POINT_H
#define POINT_H

#include <iostream>

class Point
{

public:
	Point(const double xin, const double yin, const double zin) : x(xin), y(yin), z(zin) {}
	
	double x,y,z;

	virtual void Output()
	{
      std::cout << "Point" << std::endl;
	}
	
};

class DerivedClass : Point
{
  void Output()
  {
    std::cout << "DerivedClass" << std::endl;
  }
};

#endif