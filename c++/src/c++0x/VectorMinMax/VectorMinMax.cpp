#include <iostream>
#include <vector>
#include <algorithm>

int main()
{
  // Vector initialization
  std::vector<int> v = {1,2,3};

  std::pair<std::vector<int>::iterator, std::vector<int>::iterator> minmax = std::minmax_element(v.begin(), v.end());

  std::cout << *(minmax.first) << std::endl;
  std::cout << *(minmax.second) << std::endl;

  return 0;
}
