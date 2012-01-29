#include <iostream>
#include <set>

class Point
{

public:
  double x,y;

  Point(const double xin, const double yin) : x(xin), y(yin)
  {
  }
};

bool operator<(const Point &P1, const Point &P2)
{
  if(P1.x < P2.x)
  {
    return true;
  }
  else if (P2.x < P1.x)
  {
    return false;
  }

  if (P1.y < P2.y)
  {
    return true;
  }
  else if (P2.y < P1.y)
  {
    return false;
  }

}


int main(int argc, char* argv[])
{
  Point P(0.0, 1.1);
  std::set<Point> container;
  container.insert(P);
  return 0;
}
