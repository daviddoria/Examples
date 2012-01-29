#include <iostream>
#include <string>

class Person
{
public:

  virtual void PrintSchool()
  {
    std::cout << "School name" << std::endl;
  }
};

class Lawyer : public Person
{
public:

  void PrintSchool(){std::cout << "Law school name" << std::endl;}
};

int main()
{
  Person* person = new Person;
  person->PrintSchool();
  
  Person* lawyerPerson = new Lawyer;
  lawyerPerson->PrintSchool();
  
  Lawyer* lawyer = new Lawyer;
  lawyer->PrintSchool();

  return 0;
}