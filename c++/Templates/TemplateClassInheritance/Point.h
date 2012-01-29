
#ifndef POINT_H
#define POINT_H

#include <iostream>

template <typename T>
class Point
{
protected:
	T X,Y,Z;
public:
	
	typedef double MyDouble;
	Point(const T x, const T y, const T z) : X(x), Y(y), Z(z) {}

	T getX() {return X;};
	T getY() {return Y;};
	T getZ() {return Z;};

	void Output();

};

template< typename T>
void Point<T>::Output()
{ 
	std::cout << "x: " << X << " y: " << Y << " z: " << Z << std::endl;
}

#endif
