#include <iostream>
#include <string>
#include <sstream>

int main()
{
  std::string test;
  test = "hello";
  int scan = 34;
  std::string test2;

  std::stringstream out;
  out << scan;
  test2 = out.str();
  std::cout << test << test2 << std::endl;

  std::string a = std::string("a") + std::string("b");
  return 0;
}
