
#include <iostream>
#include <vector>

template<typename T>
typename T::value_type GetValue(const T& object)
{
  return object[0];
}

template<typename T>
T GetValue(const T& object)
{
  return object;
}

template<typename T> void MyFunc(T a)
{
  std::cout << GetValue(a);
}

int main(int argc, char *argv[])
{
  MyFunc(2);

  std::vector<int> a;
  MyFunc(a);
  return 0;
}
