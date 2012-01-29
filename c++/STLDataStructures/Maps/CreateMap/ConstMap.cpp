#include <iostream>
#include <map>
#include <string>

static void TestConstKey();

int main(int argc, char *argv[])
{
  // Create a map from strings to doubles
  std::map <std::string, double> myMap;
  
  // Create a mapping from "testone" to 111
  myMap["two"] = 2;
  
  const std::map <std::string, double>& myConstMap = myMap;
  
  // std::cout << myConstMap["two"];j // Can't do this, because the value is attempted to be automatically created
  // Must do this instead:
  std::cout << myMap.find("One hundred eleven")->second;

  TestConstKey();

  return 0;
}

struct MyClass{};
void TestConstKey()
{
  // This doesn't work.
//   std::map<MyClass*, int> myMap;
//   const MyClass* test;
//   myMap[test] = 2;

  // const MyClass* is a different type than MyClass*
  std::map<const MyClass*, int> myMap;
  const MyClass* test;
  myMap[test] = 2;
}