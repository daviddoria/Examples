#include <iostream>

template <typename TParent>
class Parent
{

};

template <typename TChild>
class TypedChild : public Parent<TChild>
{
};

int main(int argc, char* argv[])
{
  Parent<int> parent;

  TypedChild<int> child;

  return 0;
}
