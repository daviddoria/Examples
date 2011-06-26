#include <iostream>
#include <map>
#include <vector>
#include <string>

#include "MyClass.h"

int main(int argc, char *argv[])
{
	MyClass A;
	//works
	/*
	std::string test = "one";
	int val = A.GetValue(test);
	*/
	
	//works
	/*
	std::string test("one");
	int val = A.GetValue(test);
	*/
	
	//works
	int val = A.GetIntVal("one");
	std::cout << val << "\n";
	
	double val2 = A.GetDoubleVal("one");
	std::cout << val2 << "\n";
	
	//works
	//int val = A.GetValue(test);
	
	
	return 0;
}