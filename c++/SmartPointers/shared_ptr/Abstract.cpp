#include <iostream>
#include <memory> // shared_ptr
class AbstractClass
{
  virtual void Test() = 0;
};

class SubClass : public AbstractClass
{
  void Test() {}
};

class MyClass
{
public:
  AbstractClass* GetObject() {return new SubClass;}
};

int main(int argc, char *argv[])
{
  MyClass myClass;
  std::shared_ptr<AbstractClass> test(myClass.GetObject());

  return 0;
}
