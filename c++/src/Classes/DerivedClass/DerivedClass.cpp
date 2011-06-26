#include <iostream>
#include <string>

class Person
{
  
  public:
    std::string Name;
    Person(){}
    
};

class Child : public Person
{
  
  public:
    std::string ParentName;
    Child(){}
};

int main(int argc, char *argv[])
{
  Person* A = new Person;
  A->Name = "TestPerson";
  std::cout << A->Name << std::endl;
  
  Child* C = new Child;
  C->Name = "TestChild";
  C->ParentName = "TestChildParent";
  std::cout << C->Name << std::endl;
  std::cout << C->ParentName << std::endl;
  
  Person *TestPerson;
  
  TestPerson = new Child;
  TestPerson->Name = "Working";
  std::cout << TestPerson->Name << std::endl;
  
  //TestPerson->ParentName = "Working";
  //std::cout << TestPerson->ParentName << std::endl;
  
  
  
  return 0;
}
