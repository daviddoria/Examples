#include <iostream>
#include <vector>

int main(int, char*[])
{

  int a = 2;
  std::cout << a << std::endl;

  typeof(a) b = 3;
  std::cout << b << std::endl;

  std::vector<typeof(a)> vec;
  vec.push_back(1);
  std::cout << vec[0] << std::endl;

  /*
  // doesn't work like this
  int testInt;
  double testDouble;
  if(typeof(testInt) == typeof(testDouble))
    {
    std::cout << "uh oh!" << std::endl;
    }
  */
  return 0;
}