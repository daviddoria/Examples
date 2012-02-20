#include <iostream>

template <typename TParent>
class Parent
{

};

class Child : public Parent<int>
{
};

int main(int argc, char* argv[])
{
  Child child;

  return 0;
}