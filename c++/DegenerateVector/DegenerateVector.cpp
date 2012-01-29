#include <iostream>
#include <vector>

template<typename T>
typename std::enable_if<std::is_fundamental<T>::value, T&>::type index(T& t, size_t)
{
  return t;
}

template<typename T>
typename T::value_type& index(T& v, size_t i)
{
  return v[i];
}

int main(int argc, char *argv[])
{
  int i;
  index(i,0);

  std::vector<int> v;
  index(v,0);

  return 0;
}
