#include <iostream>
#include <cstdlib>
#include <limits>

template<typename T>
typename std::enable_if<std::numeric_limits<T>::is_signed, T&>::type difference(T& a, T& b)
{
  return abs(a-b);
}


int main(int argc, char *argv[])
{
  std::cout << abs(5u-3u) << std::endl;
  std::cout << abs(3u-5u) << std::endl;

  unsigned int a = 5u-3u;
  std::cout << a << std::endl;

  unsigned int b = 3u-5u;
  std::cout << b << std::endl;

  int c = 3u-5u;
  std::cout << c << std::endl;

  std::cout << 4294967294 << std::endl;
  std::cout << static_cast<int>(4294967294) << std::endl;
  std::cout << static_cast<int>(4294967) << std::endl;
  return 0;
}
