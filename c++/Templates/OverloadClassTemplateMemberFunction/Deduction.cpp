#include <iostream>
#include <vector>

template <typename T>
class Simple
{
};

template <typename C>
class MyClass
{
public:
  template <typename T>
  void Output(const T& object)
  {
    std::cout << object << std::endl;
  }

  // I want this to handle MyClass<int>, MyClass<float>, MyClass<anything>
  template <typename T>
  void Output( const MyClass<T>& object)
  {
    std::cout << "special" << std::endl;
  }
};


int main(int argc, char* argv[])
{
  MyClass<double> myClass;
  myClass.Output(myClass);

  myClass.Output(2u);

  return 0;
}