#include <iostream>
#include <exception>

static void TestFunction();

class myexception: public std::exception
{
  virtual const char* what() const throw()
  {
    return "My exception happened";
  }
};

int main()
{
//   try
//   {
//     TestFunction();
//   }
//   catch (...)
//   {
//     std::cerr << "Caught an exception." << std::endl;
//   }

  TestFunction();
  return 0;
}

void TestFunction()
{
  myexception ex;
  throw ex;
}
