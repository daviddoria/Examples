#include <iostream>
#include <limits>

class Parent
{
public:
  double age;
};

class Child : public Parent
{
public:
  int age;
};

int main(int argc, char *argv[])
{
  Parent parent;
  parent.age = 20.4;
  
  std::cout << "Parent age: " << parent.age << std::endl; // Outputs 20.4
  
  Child child;
  child.age = 20.4;
  
  std::cout << "Child age: " << child.age << std::endl; // Outputs 20 (automatically casted to int in the subclass)
  
  return 0;
}
