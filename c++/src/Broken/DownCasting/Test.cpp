#include <iostream>
#include <string>

class PersonClass
{
  public:
  std::string Name;
  virtual void test(){}; //it is annoying that this has to be here...
};

class LawyerClass : public PersonClass
{
  public:
  void GoToCourt(){};
};

class DoctorClass : public PersonClass
{
  public:
  void GoToSurgery(){};
};

int main(int argc, char *argv[])
{
  
  /* 
  //this works fine
  PersonClass* person = new PersonClass;
  LawyerClass* lawyer = dynamic_cast<LawyerClass*>(person);
  lawyer->GoToCourt();
  */
  
  /*
  PersonClass* person = new PersonClass;
  if(true)
  {
    LawyerClass* lawyer = dynamic_cast<LawyerClass*>(person);
  }
  else
  {
    DoctorClass* doctor = dynamic_cast<DoctorClass*>(person);
  }
  
  if(true)
  {
    lawyer->GoToCourt();
  }
  else
  {
    doctor->GoToSurgery();
  }
  */
  
  PersonClass* person = new PersonClass;
  LawyerClass* lawyer = NULL;
  DoctorClass* doctor = NULL;
  
  if(true)
  {
    lawyer = dynamic_cast<LawyerClass*>(person);
  }
  else
  {
    doctor = dynamic_cast<DoctorClass*>(person);
  }
  
  if(true)
  {
    lawyer->GoToCourt();
  }
  else
  {
    doctor->GoToSurgery();
  }
  
  return 0;
}
