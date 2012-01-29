#include <iostream>

// You can't do this!

int main(int argc, char *argv[])
{
  //bool useInt = true;
  bool useInt = false;

  typedef int MyType;
  
  if(useInt)
    {
    typedef int MyType;
    }
  else
    {
    typedef double MyType;
    }

  MyType a = 2.1;

  std::cout << a << std::endl;
  return 0;
}
