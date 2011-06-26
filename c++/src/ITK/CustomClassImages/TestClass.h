#ifndef TESTCLASS_H
#define TESTCLASS_H

#include <iostream>
#include <string>

class TestClass
{
public:
  TestClass(){}
  TestClass(const double d, const std::string &s) : MyDouble(d), MyString(s) {}

  double MyDouble;
  std::string MyString;

};

std::ostream& operator<<(std::ostream& output, const TestClass &T);

#endif