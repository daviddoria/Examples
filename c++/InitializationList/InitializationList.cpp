#include <iostream>


class Point
{
public:
  //Point(){data[0] = 0; data[1] = 1; data[2] = 2;}
  Point(double x, double y, double z){data[0] = x; data[1] = y; data[2] = z;}
  double data[3];
};

class PointList
{
public:
  PointList() : P(1,2,3){}
  
private:
  Point P;
};


int main(int argc, char* argv[])
{
  PointList pointList;
  //std::cout << p.data[0] << " " << p.data[1] << " " << p.data[2] << std::endl;
  return 0;
}
