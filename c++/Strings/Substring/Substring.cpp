#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <iterator>
#include <vector>
 
int main(int argc, char *argv[])
{

  std::string myString = "Test123";
  
  std::cout << myString.substr(5, 7) << std::endl;
  
  // Get the last 'n' characters
  unsigned int n = 3;
  std::cout << myString.substr(myString.size() - n, myString.size()) << std::endl;
  return 0;
}
