#include <iostream>
#include <set>

struct Point
{
  float x,y;
};

struct MyComparison
{
  bool operator()(const Point p1, const Point p2) const
  {
    if((p1.x < p2.x))
    {
      return true;
    }

    if(p1.x == p2.x)
      {
      if(p1.y < p2.y)
        {
        return true;
        }
      }

    return false;
  }
};

int main(int argc, char* argv[])
{
  typedef std::set<Point, MyComparison> SetType;
  SetType s;
  std::pair<SetType::iterator, bool> result;
  Point p1;
  p1.x=10;
  p1.y=10;
  result = s.insert(p1);
  std::cout << result.second << std::endl;

  Point p2;
  p2.x=10;
  p2.y=11;
  result = s.insert(p2);
  std::cout << result.second << std::endl;

  return 0;
}
