#ifndef TESTCLASS_H
#define TESTCLASS_H

#include <vector>

class TestClass
{
	std::vector<double> TestVector;
	
	//std::vector<double> TestVector(10); //cant do this!
	
	public:
		TestClass() : TestVector(10) {}; //do this instead!
};

#endif