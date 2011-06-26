#include "Point.h"
#include "Vector.h"

////////////// Non Member Functions //////////////
ostream& operator<< (ostream& output, const Point& p)
{
	output << "(" <<  p.getX() << ", " << p.getY() << ", " << p.getZ() << ")";
	return output;
}

/////////// Operators //////////////

Point Point::operator+ (const Vector &V) const 
{  
	//Must leave this in the implementation file because geom_Vector3 is forward declared!
	return Point(X_ + V.getX(), Y_ + V.getY(), Z_ + V.getZ());  
	
}

Vector Point::operator- (const Point &P) const
{  
	//Must leave this in the implementation file because geom_Vector3 is forward declared!
	return Vector(X_ - P.getX(), Y_ - P.getY(), Z_ - P.getZ());
}

/////////// Functions /////////////
Vector Point::ToVector() const 
{
	return Vector(X_, Y_, Z_);
}