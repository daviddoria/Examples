// Note: this is very dangerous.

#include <iostream>

class MyClass
{
public:
  float a;
};

class MyOtherClass
{
public:
  float b;
};

int main(int, char *[])
{
  MyClass* testClass = new MyClass;
  testClass->a = 2.1f;

  //MyOtherClass* testOtherClass = testClass;
  MyOtherClass* testOtherClass = reinterpret_cast<MyOtherClass*>(testClass);
  std::cout << testOtherClass->b << std::endl;
  
  return 0;
}
