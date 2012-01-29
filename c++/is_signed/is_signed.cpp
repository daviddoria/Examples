#include <iostream>
#include <limits>

int main(int argc, char *argv[])
{
  std::cout << std::numeric_limits<unsigned int>::is_signed << std::endl;
  std::cout << std::numeric_limits<int>::is_signed << std::endl;
  return 0;
}