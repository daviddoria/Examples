#include <iostream>
#include <limits>

class Point
{

public:
  Point(const double xin, const double yin, const double zin) : x(xin), y(yin), z(zin) {}
  
  double x,y,z;

  virtual void Output() = 0;
	
};

class DerivedClass
{
  void Output()
  {
    std::cout << "Point Class." << std::endl;
  }
};	

int main(int argc, char *argv[])
{

  return 0;
}