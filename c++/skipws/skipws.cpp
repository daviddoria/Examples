#include <iostream>
#include <sstream>

int main () 
{
  char a, b, c;

  std::istringstream iss ("  123");
  iss >> std::skipws >> a >> b >> c;
  std::cout << a << b << c << std::endl;

  iss.seekg(0);
  iss >> std::noskipws >> a >> b >> c;
  std::cout << a << b << c << std::endl;
  return 0;
}