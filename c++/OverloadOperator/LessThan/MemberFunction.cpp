#include <iostream>
#include <set>

class Point
{

public:
  double x,y;

  Point(const double xin, const double yin) : x(xin), y(yin)
  {
  }

  bool operator<(const Point &other) const;
};

bool Point::operator<(const Point &other) const
{
  if(this->x < other.x)
  {
    return true;
  }
  else if (other.x < this->x)
  {
    return false;
  }

  if(this->y < other.y)
  {
    return true;
  }
  else if (other.y < this->y)
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
