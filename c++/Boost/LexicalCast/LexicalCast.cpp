#include <iostream>

#include <boost/lexical_cast.hpp>

int main(int argc, char* argv[])
{
  std::string stringNumber = "2.0";
  float floatNumber = boost::lexical_cast<float>(stringNumber);
  
  std::cout << floatNumber << std::endl;

  return 0;
}
