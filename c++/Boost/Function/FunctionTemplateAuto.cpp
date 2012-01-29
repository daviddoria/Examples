// int Person::GetAge()
{

}

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <iostream>

class Person
{
public:
  virtual int GetAge();
  int age;
};

class Student : public Person
{
public:
  int grade;
};

template <typename TPointer>
void OutputInformation(TPointer object)
{
  std::cout << object->age << std::endl;
  if(dynamic_cast<Student*>(object))
    {
    //std::cout << object->grade << std::endl;
    std::cout << "Object is a student." << std::endl;
    }
}

template <typename TPointer>
void OutputSomethingElse(TPointer object)
{
  std::cout << object->age << std::endl;
  if(dynamic_cast<Student*>(object))
    {
    //std::cout << object->grade << std::endl;
    std::cout << "Object is a student." << std::endl;
    }
}

void ApplyFunction(Person* p, boost::function<void(Person*)> f)
{
  if(dynamic_cast<Student*>(p))
    {
    f = &OutputInformation<Student*>;
    }
  else
    {
    f = &OutputInformation<Person*>;
    }
}

int main ()
{
  Student* student = new Student;

  boost::function<void(Person*)> f1;

  ApplyFunction(student, f1);

  return 0;
}
