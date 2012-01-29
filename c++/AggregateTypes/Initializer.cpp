#include <iostream>

// Works
// class Point
// {
// 
// public:
//   double data[3];
// };
// 
// int main(int argc, char* argv[])
// {
//   Point p = {0.0, 1.1, 2.0};
//   std::cout << p.data[0] << " " << p.data[1] << " " << p.data[2] << std::endl;
//   return 0;
// }

class Point
{
public:
  Point(){data[0] = 0; data[1] = 1; data[2] = 2;}
  double data[3];
};

int main(int argc, char* argv[])
{
  Point p = {0.0, 1.1, 2.0};
  std::cout << p.data[0] << " " << p.data[1] << " " << p.data[2] << std::endl;
  return 0;
}
