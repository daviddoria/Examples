#include <iostream>

typedef std::pair<unsigned int, unsigned int> IndexPair;

int main(int argc, char *argv[])
{
  IndexPair A;
  A.first = 2;
  A.second = 4;
  
  //std::cout << A << std::endl;
  std::cout << A.first << std::endl;
  std::cout << A.second << std::endl;
  return 0;
}
