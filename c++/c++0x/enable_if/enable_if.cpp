#include <iostream>
#include <limits>

template<typename T>
typename std::enable_if<std::is_fundamental<T>::value, T>::type index(T& t, size_t)
{
  return t;
}

// To use 
template<typename T>
class MyClass
{
  void MyFunc(const typename std::enable_if<std::is_fundamental<T>::value, T>::type dummy = T());
  void MyFunc(const typename std::enable_if<!std::is_fundamental<T>::value, T>::type dummy = T());
};

template<typename T>
void MyClass<T>::MyFunc(const typename std::enable_if<std::is_fundamental<T>::value, T>::type dummy)
{
}

template<typename T>
void MyClass<T>::MyFunc(const typename std::enable_if<!std::is_fundamental<T>::value, T>::type dummy)
{
}

int main(int argc, char *argv[])
{
//   MyClass<int> myClass;
//   myClass.MyFunc();

  int a;
  index(a, 0);
  return 0;
}