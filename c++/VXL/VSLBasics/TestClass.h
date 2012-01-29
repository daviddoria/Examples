#ifndef TESTCLASS_H
#define TESTCLASS_H

#include <iostream>
#include <string>
#include <vector>


#include <vsl/vsl_binary_io.h>

class TestClass
{
	public:
		TestClass(){}
		TestClass(const double d, const std::string &s) : MyDouble(d), MyString(s) {}
		
		double MyDouble;
		std::string MyString;
		
		
};

void vsl_b_write(vsl_b_ostream &os, TestClass const &obj);
void vsl_b_read(vsl_b_istream &is, TestClass const &obj);
void vsl_print_summary(vcl_ostream &os, TestClass const &obj);

void vsl_b_write(vsl_b_ostream &os, std::vector<TestClass> const &obj);
void vsl_b_read(vsl_b_istream &is, std::vector<TestClass> const &obj);
void vsl_print_summary(vcl_ostream &os, std::vector<TestClass> const &obj);


//std::ostream& operator<<(std::ostream& output, const TestClass &T);

#endif