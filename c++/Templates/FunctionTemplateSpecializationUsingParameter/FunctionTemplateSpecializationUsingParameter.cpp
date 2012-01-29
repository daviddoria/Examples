#include <iostream>
#include <vector>

#include "Point.h"

template <typename C>
class MyClass
{
};

template <typename T>
void Output(const T& object)
{
  std::cout << object << std::endl;
}


template <>
void Output<MyClass<C> >(const MyClass<C>& object)
{
  std::cout << "unsigned int " << object << std::endl;
}

int main(int argc, char* argv[])
{
  MyClass<double> myClass;
  Output(myClass);

  Output(2u);

  return 0;
}
