template < typename output_policy, typename language_policy >
class HelloWorld : public output_policy, public language_policy
{
  using output_policy::Print;
  using language_policy::Message;

public:
  //behaviour method
  void Run()
  {
    //two policy methods
    Print( Message() );
  }
};

#include <iostream>

class OutputPolicy_WriteToCout
{
protected:
  template< typename message_type >
  void Print( message_type message )
  {
    std::cout << message << std::endl;
  }
};

#include <string>

class LanguagePolicy_English
{
protected:
  std::string Message()
  {
    return "Hello, World!";
  }
};

class LanguagePolicy_German
{
protected:
  std::string Message()
  {
    return "Hallo Welt!";
  }
};

int main()
{
  // example 1
  typedef HelloWorld<OutputPolicy_WriteToCout, LanguagePolicy_English> my_hello_world_type;

  my_hello_world_type hello_world;
  hello_world.Run(); // Prints "Hello, World!"

  // example 2
  typedef HelloWorld<OutputPolicy_WriteToCout, LanguagePolicy_German > my_other_hello_world_type;

  my_other_hello_world_type hello_world2;
  hello_world2.Run(); // Prints "Hallo Welt!"
}
