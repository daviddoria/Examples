#include <iostream>
#include <limits>

template<typename T>
class MyClass
{
public:
  template <typename U = T>
  void MyFunc(const typename std::enable_if<std::is_fundamental<U>::value, U>::type dummy = U());
  template <typename U = T>
  void MyFunc(const typename std::enable_if<!std::is_fundamental<U>::value, U>::type dummy = U());
};

template<typename T>
template<typename U>
void MyClass<T>::MyFunc(const typename std::enable_if<std::is_fundamental<U>::value, U>::type dummy)
{
  std::cout << "POD" << std::endl;
}

template<typename T>
template<typename U>
void MyClass<T>::MyFunc(const typename std::enable_if<!std::is_fundamental<U>::value, U>::type dummy)
{
  std::cout << "NOT POD" << std::endl;
}

class Simple {};

int main(int argc, char *argv[])
{
  MyClass<int> myClass;
  myClass.MyFunc();

  MyClass<Simple> myClass2;
  myClass2.MyFunc();

  return 0;
}
