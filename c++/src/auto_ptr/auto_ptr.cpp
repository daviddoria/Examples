#include <iostream>
#include <memory>


int main(int argc, char *argv[])
{

  std::auto_ptr<int> myInt(new int);
  *myInt = 5;

  std::cout << *myInt << std::endl;

  return 0;
}