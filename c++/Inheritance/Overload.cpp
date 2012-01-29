#include <set>
#include <algorithm>

class Parent
{
public:
  float MyFunc(int a){return 0.0;}
  virtual float MyFunc(int a, int b) const = 0;
};

class Child : public Parent
{
public:
  float MyFunc(int a, int b) const{return 0.0;}
};

int main()
{
  Child child;
  //child.MyFunc(1);
  child.Parent::MyFunc(2);
  return 0;
}
