#include <iostream>
#include <vector>
#include <algorithm>

struct Point
{
  int x,y;
};

bool operator<(const Point& p1, const Point& p2)
{
  if(p1.x < p2.x)
    {
    return true;
    }
  else
    {
    return p1.y < p2.y;
    }
}


//////////////////////////
int main (int argc, char *argv[])
{
  std::vector<Point> points;
  Point a;
  points.push_back(a);
  points.push_back(a);
  points.push_back(a);

  //sort in ascending order
  std::sort(points.begin(), points.end());

  for(unsigned int i = 0; i < points.size(); i++)
  {
    //std::cout << points[i] << std::endl;
  }

  return 0;
}
