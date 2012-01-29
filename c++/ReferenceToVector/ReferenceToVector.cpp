#include <iostream>
#include <vector>

class TestClass : public std::vector<int>
{
  double TestValue;
};

int main(int, char *[])
{
  TestClass test;
  test.push_back(1);
  test.push_back(2);
  
  std::cout << test[0] << std::endl;
  
  TestClass& test2 = test;
  
  /*
  // Can't do this:
  TestClass& test2;
  test2 = test;
  */
  std::cout << test2[0] << std::endl;
  
  return 0;
}
