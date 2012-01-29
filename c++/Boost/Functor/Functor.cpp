#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <iostream>

struct MyClass
{
public:
  void operator()(const int a)
  {
    std::cout << a + b << std::endl;
  };
  int b;
};

int main ()
{
  MyClass test;
  test.b = 4;

  boost::function< void(const int) > myFunction = boost::bind<void>(test, _1);
  myFunction(3);
  //boost::function< void > myFunction = boost::bind<void>(a); // Note the () after void are necessary to avoid "incomplete type" error.

  return 0;
}
