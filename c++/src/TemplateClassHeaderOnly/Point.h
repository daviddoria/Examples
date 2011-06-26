#ifndef POINT_H
#define POINT_H

#include <iostream>
#include <vector>

template <typename T>
class Point
{
	T X,Y,Z;
public:
	Point(const T x, const T y, const T z) : X(x), Y(y), Z(z) {}

  T getX() {return X;};
	T getY() {return Y;};
	T getZ() {return Z;};

    void Output(void) { std::cout << "x: " << X << " y: " << Y << " z: " << Z << std::endl;}

};


#endif

