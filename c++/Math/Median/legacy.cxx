#include <algorithm>
#include <iostream>
#include <cmath>
#include <vector>

template<typename T>
T VectorMedian(std::vector<T> v)
{
  /*
  int n = v.size() / 2;
  std::nth_element(v.begin(), v.begin()+n, v.end());
  return v[n];
  */


  std::sort(v.begin(), v.end());

  for(unsigned int i = 0; i < v.size(); ++i)
    {
    std::cout << "i: " << i << " = " << v[i] << std::endl;
    }

  int n = v.size() / 2;
  return v[n];
}

int main(int argc, char *argv[])
{
//   std::vector<int> v;
//   v.push_back(1);
//   v.push_back(2);
//   v.push_back(3);
//
//   std::cout << VectorMedian<int>(v) << std::endl;

  std::vector<float> v;
  v.push_back(1.0);
  v.push_back(2.2);
  v.push_back(3.1);

  std::cout << VectorMedian<float>(v) << std::endl;
  return 0;
}
