#include <string>
#include <iostream>

class Person
{
  public:
  std::string Name;
  
  void CopyPerson(Person* p)
  {
    this->Name = p->Name;
  }
};

class Scientist : public Person
{

  public:
    std::string Field;
    
    void CopyScientist(Scientist* s)
    {
      this->Name = s->Name;
      this->Field = s->Field;
    }
    
    void CopyPerson(Person* p)
    {
      this->Name = p->Name;
    }
};

int main()
{
  Person* pDavid = new Person();
  pDavid->Name = "David";
  
  /* this works correctly (sets pTony's name to "David")
  Person* pTony = new Person();
  pTony->CopyPerson(pDavid);
  std::cout << "pTony name: " << pTony->Name << std::endl;
  */
  
  Scientist* sDavid = new Scientist();
  sDavid->CopyPerson(pDavid);
  std::cout << "sDavid name: " << sDavid->Name << std::endl;
  
  return 0;
}
