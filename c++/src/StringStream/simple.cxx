#include <iostream>
#include <string>
#include <sstream>

int main()
{
  std::stringstream TestStream1;
  float temp;

  std::cout << std::endl;
  TestStream1.clear();
  TestStream1.str("");
  TestStream1 << "7";
  std::cout << TestStream1.str() << std::endl;
  TestStream1 >> temp;
  std::cout << temp << std::endl;

  TestStream1.clear();
  TestStream1.str("");
 
  TestStream1 << "8";
  std::cout << TestStream1.str() << std::endl;
  TestStream1 >> temp;
  std::cout << temp << std::endl;

  return 0;
}
