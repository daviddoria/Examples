#ifndef POINT_H
#define POINT_H

class Point
{

public:
	Point(const double xin, const double yin, const double zin) : x(xin), y(yin), z(zin) {}
	
	double x,y,z;
	
};

#endif