#include <iostream>

template <typename TParent>
class Parent
{

};

class ParentNoTemplate
{
public:
  void TestParentNoTemplate(){}
};

class Child : public Parent<int>, public ParentNoTemplate
{
public:
  void Test()
  {
    TestParentNoTemplate();
  }
};

template <typename TChild>
class TypedChild : public Parent<TChild>
{
public:
  void Test()
  {
    TestParentNoTemplate();
  }
};

int main(int argc, char* argv[])
{
  TypedChild<int> c;

  Child c;
  c.Test();

  return 0;
}
