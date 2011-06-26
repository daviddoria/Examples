#include <iostream>

//forward declaration
class B;

class A
{
private:
  friend class B;

  int aData;

public:

};

class B
{
public:
  void Output(A a)
  {
    std::cout << a.aData << std::endl;
  }
};

int main(int argc, char *argv[])
{
  A myAClass;
  B myBClass;
  myBClass.Output(myAClass);

  return 0;
}