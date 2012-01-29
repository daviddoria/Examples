#include <iostream>

class Point
{

public:
  double x,y,z;

  Point(){}
  Point(const double xin, const double yin, const double zin) : x(xin), y(yin), z(zin) {}

  friend std::ostream& operator<<(std::ostream& output,  const Point &P);

};

std::ostream& operator<<(std::ostream& output, const Point &P)
{
  output << "Point: " << P.x << " " << P.y << " " << P.z << std::endl;

  return output;
}

int main(int argc, char* argv[])
{
  Point P(0.0, 1.1, 2.0);
  std::cout << P << std::endl;

  return 0;
}
