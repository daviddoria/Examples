#include <iostream>
#include <vector>
#include <numeric>

int main (int argc, char *argv[])
{
  std::vector<int> vec;
  vec.push_back(1);
  vec.push_back(2);
  vec.push_back(3);
  
  int vecSum = std::accumulate(vec.begin(), vec.begin() + vec.size(), 0);

  std::cout << "sum: " << vecSum << std::endl;
  
  return 0;
}
