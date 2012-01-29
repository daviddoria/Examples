#include <algorithm>
#include <iostream>
#include <cmath>
#include <vector>

template<typename T>
T VectorMedian(std::vector<T> v)
{
  int n = v.size() / 2;
  std::nth_element(v.begin(), v.begin()+n, v.end());
  return v[n];
}

int main(int argc, char *argv[])
{
//   std::vector<int> v;
//   v.push_back(1);
//   v.push_back(3);
//   v.push_back(2);

//   std::cout << VectorMedian<int>(v) << std::endl;

  std::vector<float> v;
  v.push_back(1.0);
  v.push_back(3.1);
  v.push_back(2.2);

  std::cout << VectorMedian<float>(v) << std::endl;
  return 0;
}
