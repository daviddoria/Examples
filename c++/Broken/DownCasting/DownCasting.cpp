#include <iostream>
#include <string>

class PersonClass
{
  public:
  std::string Name;
  virtual void test(){};
};

class LawyerClass : public PersonClass
{
  public:
  std::string School;
  void GoToCourt(){};
};

void Works();
void DoesntWork();
void SubclassFunction();

int main(int argc, char *argv[])
{
  //Works();
  //DoesntWork();
  SubclassFunction();
  
  return 0;
}

void Works()
{
  PersonClass* Person = new LawyerClass;
  Person->Name = "test";
  std::cout << Person->Name;
  
  LawyerClass* Lawyer = static_cast<LawyerClass*>(Person);
  Lawyer->School = "testSchool";
  
  std::cout << Lawyer->Name << " " << Lawyer->School << std::endl;
  
}

void DoesntWork()
{
  
  PersonClass* Person = new PersonClass;
  Person->Name = "test";
  std::cout << Person->Name;
  
  
  LawyerClass* Lawyer = static_cast<LawyerClass*>(Person);
  std::cout << Lawyer->Name;
  Lawyer->School = "testSchool";
  
}

void SubclassFunction()
{
/* working  
  PersonClass* person = new PersonClass;
  LawyerClass* lawyer = dynamic_cast<LawyerClass*>(person);
  lawyer->GoToCourt();
  */
  
  PersonClass* person = new PersonClass;
  if(true)
  {
  LawyerClass* lawyer = dynamic_cast<LawyerClass*>(person);
  }
  
  // ...
  
  //lawyer->GoToCourt();
  
}
