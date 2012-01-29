#include <iostream>
#include <vector>

template <typename T>
void Output(const T& object)
{
  std::cout << object << std::endl;
}

template <typename C>
class MyClass
{
};

// Handles MyClass<int>, MyClass<float>, MyClass<anything>
template <typename T>
void Output( const MyClass<T>& object)
{
  std::cout << "Special " << std::endl;
}

int main(int argc, char* argv[])
{
  MyClass<double> myClass;
  //Output(myClass);
  //Output<MyClass<double> >(myClass);
  Output<MyClass<double> >(myClass);

  Output(2u);

  return 0;
}