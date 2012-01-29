#include <iostream>
#include <memory>
#include <stdexcept>

// Must add '-std=c++0x' to CXX_FLAGS

class MyClass
{
public:
  MyClass(int a, int b) {}
};

class AbstractClass
{
  virtual void Test() = 0;
};

static void TestBasic();
static void TestClass();
static void TestDefaultConstructor();
static void TestAbstractPointer();

int main(int argc, char *argv[])
{
  TestBasic();
  TestClass();
  TestDefaultConstructor();
  TestAbstractPointer();
  return 0;
}

void TestBasic()
{

  std::shared_ptr<int> myInt(new int);
  *myInt = 5;

  std::cout << *myInt << std::endl;

  int* test = myInt.get(); // Get a raw pointer

  std::cout << *test << std::endl;

  std::shared_ptr<int> test2;
  test2 = std::make_shared<int>(6);

  std::cout << *test2 << std::endl;

  std::shared_ptr<int> test3;
  test3 = std::shared_ptr<int>(new int(6));
}

void TestClass()
{
  std::shared_ptr<MyClass> test;
  test = std::make_shared<MyClass>(6, 3);
}

void TestDefaultConstructor()
{
  std::shared_ptr<int> test;
  // This works in both Debug and Release mode
  if(test != NULL)
    {
    throw std::runtime_error("Default pointer should be NULL!");
    }

}

void TestAbstractPointer()
{
  std::shared_ptr<AbstractClass> test;
}
