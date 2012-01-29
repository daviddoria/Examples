//  dynamic_cast is always successful when we cast a class to one of its base classes:

#include <iostream>

class Person
{
public:
  int age;
  virtual int GetAge() {return age;} // Without this, error "source type is not polymorphic".
};

class Student : public Person
{
public:
  int grade;
};

void Upcast();
void Downcast();

int main(int, char *[])
{
  //Upcast();
  Downcast();
  return 0;
}

void Upcast()
{
  Student* myStudent = new Student;
  myStudent->grade = 6;
  myStudent->age = 10;

  Person* studentPerson = dynamic_cast<Person*>(myStudent);
  std::cout << studentPerson->age << std::endl;
  
}

void Downcast()
{
  Person* myPerson = new Student;
  myPerson->age = 10;

  Student* personStudent = dynamic_cast<Student*>(myPerson);
  if(!personStudent)
    {
    std::cout << "cast failed!" << std::endl;
    }
  else
    {
    std::cout << personStudent->age << std::endl;
    personStudent->grade = 12;
    std::cout << personStudent->grade << std::endl;
    }
}
