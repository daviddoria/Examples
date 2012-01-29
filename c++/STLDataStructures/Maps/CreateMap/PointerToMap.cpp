#include <iostream>
#include <map>

class TestClass
{
  TestClass()
  {
    // (*myMap)[7] = 1.0f; // works
    (*this->myMap)[7] = 1.0f;
  }

private:
  typedef std::map<int, float> MapType;
  MapType* myMap;
};

int main(int argc, char *argv[])
{
  typedef std::map<int, float> MapType;
  MapType* myMap = new MapType;
  (*myMap)[7] = 1.0f;

  // Note these do not work:
  //*(myMap)[7] = 1.0f;
  //*myMap[7] = 1.0f;

  return 0;
}
