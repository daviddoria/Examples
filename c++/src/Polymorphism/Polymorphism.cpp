#include <iostream>
#include <string>

class Person
{
public:
  std::string Name;
  virtual void PrintSchool() = 0;
};

class Lawyer : public Person
{
public:
  std::string School;
  void PrintSchool(){std::cout << School << std::endl;}
};

int main()
{
  Person* lawyer = new Lawyer;

  lawyer->Name = "Erin";
  std::cout << lawyer->Name << std::endl;

  dynamic_cast<Lawyer*>(lawyer)->School = "Columbia";
  dynamic_cast<Lawyer*>(lawyer)->PrintSchool();

  return 0;
}