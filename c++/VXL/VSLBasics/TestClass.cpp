#include "TestClass.h"
/*
std::ostream& operator<<(std::ostream& output, const TestClass &T)
{
	output << "Test class: " << T.MyDouble << " " << T.MyString << std::endl;
	
	return output;
}
*/

void vsl_b_write(vsl_b_ostream &os, TestClass const &obj)
{}

void vsl_b_read(vsl_b_istream &is, TestClass const &obj)
{}

void vsl_print_summary(vcl_ostream &os, TestClass const &obj)
{}


void vsl_b_write(vsl_b_ostream &os, std::vector<TestClass> const &obj)
{}

void vsl_b_read(vsl_b_istream &is, std::vector<TestClass> const &obj)
{}

void vsl_print_summary(vcl_ostream &os, std::vector<TestClass> const &obj)
{}