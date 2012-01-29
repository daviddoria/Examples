// See also Mixin
#include <iostream>
#include <string>

template < typename language_policy >
class HelloWorld
{
public:
  //behaviour method
  void Run()
  {
    //two policy methods
    std::cout << language_policy::Message() << std::endl;
  }
};



struct LanguagePolicy_English
{
  static std::string Message()
  {
    return "Hello, World!";
  }
};

struct LanguagePolicy_German
{
  static std::string Message()
  {
    return "Hallo Welt!";
  }
};

int main()
{
  // example 1
  typedef HelloWorld<LanguagePolicy_English> my_hello_world_type;

  my_hello_world_type hello_world;
  hello_world.Run(); // Prints "Hello, World!"

  // example 2
  typedef HelloWorld<LanguagePolicy_German > my_other_hello_world_type;

  my_other_hello_world_type hello_world2;
  hello_world2.Run(); // Prints "Hallo Welt!"
}
