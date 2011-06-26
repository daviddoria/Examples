#include <vector>
#include <iostream>
#include <cmath>
#include <cstdlib>

using namespace std;

class Point
{
public:
  double X,Y,Z;
  Point(const double x, const double y, const double z) : X(x), Y(y), Z(z) {}

};

class Object
{
public:
  Object(){}
  ~Object()
  {
    for(int i = 0; i < Points.size(); i++)
    {
      delete Points[i];
    }
  }
  
  vector<Point*> Points;

  void MakePoints()
  {
    for(int i = 0; i < 100; i++)
    {
      Points.push_back(new Point(rand(), rand(), rand()));
    }
  }
};

int main(int argc, char* argv[])
{

  Object TestObject;
  return 0;
}
