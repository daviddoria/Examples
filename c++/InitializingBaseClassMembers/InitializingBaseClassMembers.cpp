#include <iostream>
#include <vector>

struct ParentClass
{
  double parentData;

  ParentClass() : parentData(2) {}
};

struct ChildClass : public ParentClass
{
  double childData;

  //ChildClass() : childData(0) {}

  // could also do
  ChildClass() : ParentClass(), childData(0) {}

  // does not work
  //ChildClass() : childData(0), parentData(0) {}
};

int main(int, char *[])
{


  return 0;
}
