#include <stdio.h>
#include <stddef.h>

class MyClass
{
public:
  MyClass(const int* const input) : Value(input){}

  MyClass(const MyClass& other) : Value(other.Value)
  {
    //*(this->Value) = *(other.Value);
  }

private:
  const int* const Value;

};

int main ()
{
  int* test = new int;
  MyClass a(test);
  MyClass b(a);

  return 0;
}
