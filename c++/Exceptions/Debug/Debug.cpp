#include <exception>
#include <iostream>
#include <string>

class Test
{
public:
  void MyFunction()
  {
    std::cout << "Enter" << std::endl;
    throw 1;
  }
};

int main( void )
{
  try
  {
  Test t;
  t.MyFunction();
  
  }
  catch (...)
  {
    std::cout << "something" << std::endl;
  }
  return 0;
}
