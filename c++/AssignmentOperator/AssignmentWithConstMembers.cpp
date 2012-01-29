#include <stdio.h>
#include <stddef.h>

// YOU CANNOT DO THIS. See Copy Constructor instead.
class MyClass
{
public:
  MyClass(const int* const input) : Value(input){}

  MyClass& operator= (const MyClass& other)
  {
    if (this != &other)
    {
      *(this->Value) = *(other.Value);
    }

    return *this;
  }

private:
  const int* const Value;

};

int main ()
{
  int* test = new int;
  MyClass a(test);
  MyClass b = a;

  return 0;
}
