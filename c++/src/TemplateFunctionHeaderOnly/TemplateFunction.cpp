#include <iostream>
#include <vector>

#include "Tools.h"
#include "Point.h"

void TestOutput();
void TestDisplay();

int main(int argc, char* argv[])
{
	//TestOutput();
	TestDisplay();
	
	return 0;
}

void TestOutput()
{
	std::vector<unsigned int> A;
	for(unsigned int i = 0; i < 10; i++)
		A.push_back(i * 1.3);

	Output(A);

	std::vector<double> B;
	for(unsigned int i = 0; i < 10; i++)
		B.push_back(i * 1.3);

	Output(B);
}

void TestDisplay()
{
	std::vector<unsigned int> A;
	for(unsigned int i = 0; i < 10; i++)
		A.push_back(i * 1.3);

	Display(A);

	std::vector<Point> Points;
	for(unsigned int i = 0; i < 10; i++)
		Points.push_back(Point(0.0, 1.0, 2.0));

	Display(Points);
}
