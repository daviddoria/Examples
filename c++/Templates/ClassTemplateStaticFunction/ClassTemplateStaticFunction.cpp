#include <iostream>

template<typename T>
class Point
{
public:
  static void MyFunction() {std::cout << "hi" << std::endl;}
};


int main(int argc, char* argv[])
{
  Point<int>::MyFunction();
  
  return 0;
}