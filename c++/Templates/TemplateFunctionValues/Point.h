#ifndef POINT_H
#define POINT_H

#include <iostream>

class Point
{
	public:
	double x,y,z;

	Point(const double X, const double Y, const double Z) : x(X), y(Y), z(Z){}

	Point operator+(const Point &P) const;
};

std::ostream& operator<<(std::ostream& output, const Point &P);


#endif
