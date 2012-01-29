#include <iostream>

template<typename T>
class Parent
{
public:
  void TestParent(){}
};

class Child : public Parent<int>
{
public:
  void Test()
  {
    TestParent();
  }
};

int main(int argc, char* argv[])
{
  Child c;

  return 0;
}
