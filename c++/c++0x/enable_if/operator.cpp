#include <iostream>
#include <limits>

class Simple
{
public:
  double operator[](int index){return 1.2;}
};

template<typename T>
class MyClass
{
public:
  
  void MyFunc(const typename std::enable_if<T::operator[]>::type* dummy = 0);

};

template<typename T>
void MyClass<T>::MyFunc(const typename std::enable_if<T::operator[]>::type* dummy)
{
  std::cout << "POD" << std::endl;
}


int main(int argc, char *argv[])
{
  MyClass<int> myClass;
  myClass.MyFunc();

  MyClass<Simple> myClass2;
  myClass2.MyFunc();

  return 0;
}
