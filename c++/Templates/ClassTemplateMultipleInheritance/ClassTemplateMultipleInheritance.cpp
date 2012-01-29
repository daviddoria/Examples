#include <iostream>

template<typename T>
class Parent
{
public:
  void TestParent(){}
};

template<typename T>
class Child : public Parent<T>
{
public:
  using Parent<T>::TestParent;
  void TestChild()
  {
    //TestParent(); error: there are no arguments to ‘TestParent’ that depend on a template parameter, so a declaration of ‘TestParent’ must be available
    // (because  because the compiler doesn't know that all specializations will have the member)
    TestParent();
  }
};

int main(int argc, char* argv[])
{
  Child<int> c;

  return 0;
}
