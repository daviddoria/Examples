#include <iostream>
#include <fstream>
#include <vbl/vbl_array_2d.h>

void TestBool();
void Test2DArray();

int main()
{	
	//Test2DArray();
	TestBool();
	return 0;
}


void Test2DArray()
{
	vbl_array_2d<double> Matrix(5,5);
	std::cout << Matrix << std::endl;

	Matrix(0,0) = 2.0;
	Matrix(3,3) = 3.0;

	std::cout << Matrix << std::endl;
	
	std::string Filename = "test.txt";
	std::ofstream fout(Filename.c_str());
	
	fout << Matrix << std::endl;
	std::cout << "Wrote file." << std::endl;
	fout.close();

}

void TestBool()
{
	vbl_array_2d<bool> Matrix(5,5,false);
	Matrix(0,0) = true;
	std::cout << Matrix << std::endl;

}