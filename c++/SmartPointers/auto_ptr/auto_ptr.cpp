#include <iostream>
#include <memory>

// auto_ptr is deprecated in c++0x.
// It is simply a shared_ptr that can only have reference count <= 1.

int main(int argc, char *argv[])
{
  std::auto_ptr<int> myInt(new int);
  *myInt = 5;

  int* test = myInt.get(); // Get the raw pointer from the smart pointer.
  
  std::cout << *myInt << std::endl;

  return 0;
}
