#include <iostream>
#include <vector>
#include <algorithm>

int main()
{
  {
  std::vector<int> v;
  v.push_back(1);
  v.push_back(2);
  v.push_back(3);

  std::cout << *(std::min_element(v.begin(), v.end())) << std::endl;
  std::cout << *(std::max_element(v.begin(), v.end())) << std::endl;
  }
  
  std::vector<float> v;
  float a = 1e-10;
  v.push_back(a);
  v.push_back(2);
  v.push_back(3);

  std::cout << *(std::min_element(v.begin(), v.end())) << std::endl;
  std::cout << *(std::max_element(v.begin(), v.end())) << std::endl;

  return 0;
}
