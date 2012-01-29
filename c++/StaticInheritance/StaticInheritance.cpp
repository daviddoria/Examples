#include <iostream>

class Parent
{
public:
  static double GetValue();
};

double Parent::GetValue()
{
  return 2.0;
}

class ChildWithReimplement : public Parent
{
public:
  static double GetValue();
};

double ChildWithReimplement::GetValue()
{
  double parentValue = Parent::GetValue();
  return parentValue + 3.0;
}

class Child : public Parent
{

};


int main(int argc, char* argv[])
{
  std::cout << "parent: " << Parent::GetValue() << std::endl;
  
  std::cout << "child: " << Child::GetValue() << std::endl;
  
  std::cout << "ChildWithReimplement: " << ChildWithReimplement::GetValue() << std::endl;
  
  return 0;
}
