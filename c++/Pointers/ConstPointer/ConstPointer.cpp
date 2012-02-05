#include <iostream>

class Person
{
public:
  // int * GetValue() const {return &Value;}
  const int * GetValue() const {return &Value;}
  
  int Value;
};

int main()
{
  Person* A = new Person;

  return 0;
}