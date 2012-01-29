#include <iostream>
#include <string>

class Person
{
  public:
  std::string Name;
  
  void Test()
  {
    if(this)
    {
      std::cout << "alive." << std::endl;
    }
    else
    {
      this = new Person;
    }
  }
};

int main(int argc, char* argv[])
{
  Person* David = new Person;
  David->Test();
  
  return 0;
}
