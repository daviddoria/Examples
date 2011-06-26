#include <iostream>
#include <functional> //hash


int main(int argc, char* argv[])
{
  std::hash<const char*> H;
  std::cout << "foo -> " << H("foo") << std::endl;
  std::cout << "bar -> " << H("bar") << std::endl;

  return 0;
}
