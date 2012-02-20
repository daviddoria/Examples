#include <iostream>
#include <map>
#include <string>
#include <algorithm>

struct MyClass
{
//public:

  MyClass(const std::string& str) : testString(str){}
private:
  std::string testString;
};

int main (int argc, char *argv[]) 
{
  // Can't do this - the map is always sorted by keys. Since we want to sort by values, we must reverse the map as shown below.
//   std::map<MyClass, float> myMap;
//   myMap[MyClass("test")] = 1.0;
//   myMap[MyClass("test2")] = 2.0;

  std::map<float, MyClass> myMap;
  myMap.insert(std::pair<float, MyClass>(1.0, MyClass("test")));
  myMap.insert(std::pair<float, MyClass>(2.0, MyClass("test1")));

  return 0;
}
