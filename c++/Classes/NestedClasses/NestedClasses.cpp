#include <iostream>

class Parent
{
  friend class Child;
  int name;
  class Child
  {
    int school;
    void test(Parent* parent)
    {
      //std::cout << this->name; // this doesn't work - Child doesn't have access to Parent's members
      std::cout << parent->name;
    }
  };
};

int main(int argc, char *argv[])
{
  Parent MyParent;
  
  return 0;
}
