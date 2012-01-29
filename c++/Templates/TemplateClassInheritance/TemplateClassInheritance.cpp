#include <iostream>
#include <vector>

//#include "Point.h"
#include "Point3D.h"

int main(int argc, char* argv[])
{
	/*
	Point<double> A(1.2, 3.4, 5.6);
	A.Output();
	
	Point<unsigned int> B(4, 5, 8);
	B.Output();
	*/
	
	Point3D<double> C(1.2,2.3,4.5);
	C.Output();
	C.Output3D();

	return 0;
}
