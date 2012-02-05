#include <iostream>

class Person
{
public:
  int Value;
};

int main()
{
  Person* A = new Person;

  Person* const B = new Person;

  return 0;
}