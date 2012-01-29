#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <iostream>

template <typename T>
void MyFunction(T value)
{
  std::cout << value << std::endl;
}

int main () 
{
  boost::function<void(int)> f = &MyFunction<int>;

  f(11);

  return 0;
}
