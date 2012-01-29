#include <iostream>
#include <cstdlib>
#include <limits>

// If T is a signed type, just take the absolute value of the difference
template<typename T>
typename std::enable_if<std::numeric_limits<T>::is_signed, T>::type difference(const T& a, const T& b)
{
  return abs(a-b);
}

// If T is a unsigned type
template<typename T>
typename std::enable_if<!std::numeric_limits<T>::is_signed, T>::type difference(const T& a, const T& b)
{
  return std::max(a,b) - std::min(a,b);
}

int main(int argc, char *argv[])
{
  std::cout << difference(5u,3u) << std::endl;
  std::cout << difference(3,5) << std::endl;

  return 0;
}
