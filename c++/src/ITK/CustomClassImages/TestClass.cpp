#include "TestClass.h"

std::ostream& operator<<(std::ostream& output, const TestClass &T)
{
	output << "Test class: " << T.MyDouble << " " << T.MyString << std::endl;
	
	return output;
}