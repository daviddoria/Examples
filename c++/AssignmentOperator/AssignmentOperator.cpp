#include <stdio.h>
#include <stddef.h>

class MyClass
{
public:
  MyClass& operator= (const MyClass& other)
  {
    if (this != &other)
    {
      this->a = other.a;
    }
    
    return *this;
  }

private:
  int a;
};


int main ()
{
  MyClass a;
  MyClass b = a;

  return 0;
}
