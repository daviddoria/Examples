#include <iostream>
#include <vector>

template <typename T>
class Simple
{
};

// template <typename T>
// class MyClass
// {
// public:
// 
//   void Output(const T& object)
//   {
//     std::cout << "NON-MyClass" << std::endl;
//   }
// 
//   // I want this to handle MyClass<int>, MyClass<float>, MyClass<anything>
//   void Output( const MyClass<T>& object)
//   {
//     std::cout << "MyClass" << std::endl;
//   }
// };

template <typename T>
class MyClass
{
public:

  template<typename U>
  void Output(const U& object)
  {
    std::cout << "NON-Simple" << std::endl;
  }

  // Using this, this function always "wins" in the deduction because it is not a template.
//   void Output(const T& object)
//   {
//     std::cout << "NON-Simple" << std::endl;
//   }

  // This handles MyClass<int>, MyClass<float>, MyClass<anything>
  template<typename U>
  void Output( const Simple<U>& object)
  {
    std::cout << "Simple" << std::endl;
  }
};


int main(int argc, char* argv[])
{
  MyClass<double> myClass;
  myClass.Output(2.0f);

  Simple<int> mySimple;
  MyClass<Simple<int> > myClass2;
  myClass2.Output(mySimple);

  return 0;
}
