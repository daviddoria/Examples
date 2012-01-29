#include "Vector.h"

ostream& operator << (ostream &output, const Vector &v)
{
	output << " <" << v.getX() << ", " << v.getY() << ", " << v.getZ() << ">";
	return output;
}


Vector Vector::Cross(const Vector &V2) const
{
	double v2x = V2.getX(), v2y = V2.getY(), v2z = V2.getZ();
	return Vector( Y_ * v2z - Z_ * v2y,
		     Z_ * v2x - X_ * v2z,
			X_ * v2y - Y_ * v2x);	
}