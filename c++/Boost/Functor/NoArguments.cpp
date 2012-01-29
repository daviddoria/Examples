#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <iostream>

struct MyClass
{
public:
  void operator()()
  {
    std::cout << "Test" << std::endl;
  };
};

int main ()
{
  MyClass a;

  boost::function< void() > myFunction = boost::bind<void>(a);
  myFunction();
  //boost::function< void > myFunction = boost::bind<void>(a); // Note the () after void are necessary to avoid "incomplete type" error.

  return 0;
}
