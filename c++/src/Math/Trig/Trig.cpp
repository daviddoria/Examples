#include <cmath>
#include <iostream>

void TestTan();
void TestTand();
void TestAtan();

int main()
{
  //TestTan();
  //TestTand();
  TestAtan();
  return 0;
}

void TestAtan()
{
  /*
  // What is this??
  double x1 = 0;
  double y1 = 0;
  double x2 = 100;
  double y2 = 100;

  std::cout << 180./3.14159 * atan((x2-x1)/(y2-y1)) << std::endl;
  */
  
  // synatx is atan(y/x)
  // This produces 0 for both 0/1 and 0/-1 ! Should use atan2
  std::cout << std::endl << " atan: " << std::endl;
  std::cout << 180./3.14159 * atan(1/0.00001) << std::endl; // (0,1) should have angle 90
  std::cout << 180./3.14159 * atan(0/1) << std::endl; // (1,0) should have angle 0
  std::cout << 180./3.14159 * atan(0/-1) << std::endl; // (-1,0) should have angle 180
  std::cout << 180./3.14159 * atan(-1/0.00001) << std::endl; // (0,-1) should have angle -90
  
  // synatx is atan2(y,x)
  std::cout << std::endl << " atan2: " << std::endl;
  std::cout << 180./3.14159 * atan2(1, 0) << std::endl; // (0,1) should have angle 90
  std::cout << 180./3.14159 * atan2(0, 1) << std::endl; // (1,0) should have angle 0
  std::cout << 180./3.14159 * atan2(0, -1) << std::endl; // (-1,0) should have angle 180
  std::cout << 180./3.14159 * atan2(-1, 0) << std::endl; // (0,-1) should have angle -90
}

void TestTan()
{

}

void TestTand()
{
  //std::cout << "tand(10) = " << tand(10) << std::endl;
}
