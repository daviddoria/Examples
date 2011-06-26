
#ifndef POINT3D_H
#define POINT3D_H

#include "Point.h"

template <typename T>
class Point3D : public Point<T>
{
	
	public:
		Point3D(const T x, const T y, const T z) : Point<T>(x,y,z) {}

		void Output3D();
};

template< typename T>
void Point3D<T>::Output3D()
{
	std::cout << "X: " << this->X << std::endl;
	typename Point<T>::MyDouble a;
	
}

#endif


