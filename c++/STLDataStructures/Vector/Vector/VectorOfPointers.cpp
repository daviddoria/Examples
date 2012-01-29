#include <iostream>
#include <vector>
#include <algorithm>

class MyClass
{

};

int main(int argc, char* argv[])
{
  std::vector<MyClass> test1;
  std::vector<MyClass*> test2;
  std::vector<const MyClass*> test3;
  // std::vector<const MyClass* const> test4; // Can't do this - the pointers can't not change - they aren't setup yet!

  return 0;
}
