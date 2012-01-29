#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric> // for accumulate()

template<typename T>
T VectorMedian(std::vector<T> &v)
{
  
  int n = v.size() / 2;
  std::nth_element(v.begin(), v.begin()+n, v.end());
  return v[n];
  

  
//   std::sort(v.begin(), v.end());
//   int n = v.size() / 2;
//   return v[n];
}

template<typename T>
T VectorAverage(std::vector<T> &v)
{
  T vecSum = std::accumulate(v.begin(), v.end(), 0);
  
  return vecSum / static_cast<T>(v.size());
}

int main()
{
  std::vector<float> v;
  v.push_back(1);
  v.push_back(2);
  v.push_back(3);
  v.push_back(4);

  std::cout << "Average: " << VectorAverage<float>(v) << std::endl;
  std::cout << "Median: " << VectorMedian<float>(v) << std::endl;

  return 0;
}
