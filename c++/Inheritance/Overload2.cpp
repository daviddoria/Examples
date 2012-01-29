#include <set>
#include <algorithm>

class Parent
{
public:
  virtual float MyFunc(int a, int b) const = 0;
  float MyFunc(int a){return 0.0;}
};

class Child : public Parent
{
public:
  float MyFunc(int a, int b) const{return 0.0;}
  using Parent::MyFunc; // This "unhides" the one parameter version
};

int main()
{
  Child child;
  child.MyFunc(1);

  return 0;
}


