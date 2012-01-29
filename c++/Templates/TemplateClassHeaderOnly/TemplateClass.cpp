#include "Point.h"

int main(int argc, char* argv[])
{
	Point<double> A(1.2, 3.4, 5.6);
	A.Output();
	
	Point<unsigned int> B(4, 5, 8);
	B.Output();

	return 0;
}
