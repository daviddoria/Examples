#include <iostream>

class TestClass
{
public:

  // TestClass(){} // Can't do this - unitialized reference member
  
  TestClass(float& refMember, float& member) : RefMember(refMember), Member(member)
  {

  }
  
  void ChangeValues()
  {
    this->RefMember += 1;
    this->Member += 1;
  }
  
  float& RefMember;
  float Member;
};

int main(int, char *[])
{
  float refMember = 2.0;
  float member = 2.0;
  TestClass test(refMember, member);

  std::cout << "member: " << member << std::endl;
  std::cout << "test.Member: " << test.Member << std::endl;
  std::cout << "refMember: " << refMember << std::endl;
  std::cout << "test.refMember: " << test.RefMember << std::endl;

  test.ChangeValues();

  std::cout << std::endl;
  
  std::cout << "member: " << member << std::endl;
  std::cout << "test.Member: " << test.Member << std::endl;
  std::cout << "refMember: " << refMember << std::endl;
  std::cout << "test.refMember: " << test.RefMember << std::endl;

  return 0;
}
