#include <iostream>
#include <map>

struct MyType
{
  float x;
  float y;
};

bool operator<(const MyType& object1, const MyType& object2)
{
  return object1.x < object2.x;
}

int main(int argc, char *argv[])
{
  // Create a map from strings to doubles
  std::map <MyType, double> myMap; // <key, value>

  MyType a;
  
  myMap[a] = 2;

  return 0;
}
