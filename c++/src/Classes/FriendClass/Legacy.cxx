#include <iostream>

//forward declaration
class B;

class A
{
  private:
    friend class B;
    int a;

  public:
    A(int aA) : a(aA) { };

};

class B
{
  private:

  public:
    int getAa(const A& aObj) { return aObj.a; };
};

int main(int argc, char *argv[])
{
  A objA(42);
  B objB;
  std::cout << "Value of A.a is " << objB.getAa(objA) << std::endl;
  return 0;
}