#include <iostream>
#include <set>
#include <vector>
#include <cstdlib>

#include <algorithm>

// NOTE: You cannot do this because references are not assignable.
class MyClass
{
public:
  int a;
};

int main(int argc, char* argv[])
{
  MyClass myClass;
  myClass.a = 3;
  
  // Create a set
//   std::set<MyClass&> mySet;
//   mySet.insert(myClass);

  return 0;
}
