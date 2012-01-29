#include <iostream>
#include <limits>

template<typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type index(T& t, size_t)
{
  return t;
}


int main(int argc, char *argv[])
{
  int a;
  index(a, 0);
  return 0;
}