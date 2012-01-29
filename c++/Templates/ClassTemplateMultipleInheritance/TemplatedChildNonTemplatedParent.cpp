#include <iostream>

class Parent
{
public:
  void TestParent(){}
};

template <typename TChild>
class TypedChild : public Parent
{
public:
  void Test()
  {
    TestParent();
  }
};

int main(int argc, char* argv[])
{
  TypedChild<int> c;

  return 0;
}
