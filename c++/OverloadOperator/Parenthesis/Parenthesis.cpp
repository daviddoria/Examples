#include <iostream>
#include <set>

class Point
{

public:
  double x,y;

//   double operator()()
//   {
//   }
  
  template <typename T>
  double operator()(T a)
  {
  }
};

int main(int argc, char* argv[])
{
  Point myPoint;
  //std::cout << p() << std::endl;
  int a;
  std::cout << myPoint(a) << std::endl;
  // std::cout << myPoint<int>(a) << std::endl; // Can't do this
  std::cout << myPoint.operator()<int>(a) << std::endl;
  return 0;
}
