#include <iostream>

template <typename TParent>
class Parent
{

};

class Child : public Parent<int>
{
};

template <typename TChild>
class TypedChild : public Parent<TChild>
{
};

int main(int argc, char* argv[])
{
  Parent<int> a;
  Child b;

  TypedChild<int> c;
  
  return 0;
}