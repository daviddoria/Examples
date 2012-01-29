#include <iostream>

template<typename T>
class Simple
{
};

// template <template <typename> class T>
// void MyFunc(const T& )
// {
//   std::cout << "Special" << std::endl;
// }

template <typename T>
void MyFunc(const Simple<T>& )
{
  std::cout << "Special" << std::endl;
}

template <typename T>
void MyFunc(const T& )
{
  std::cout << "Normal" << std::endl;
}

int main(int, char*[])
{
  int a;
  MyFunc(a);

  Simple<int> b;
  MyFunc(b);

  MyFunc<Simple<int> >(b);
  
  return 0;
}
