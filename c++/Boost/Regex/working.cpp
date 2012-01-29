#include <iostream>
#include <sstream>
#include <vector>
#include <boost/regex.hpp>

// This example extracts X and Y from ( X , Y ), (X,Y), (X, Y), etc.


struct Point
{
   int X;
   int Y;
   Point(int x, int y): X(x), Y(y){}
};

typedef std::vector<Point> Polygon;

int main()
{
  Polygon poly;
  std::string s = "Polygon: (1,2), (3,4), (5,6)";

  const boost::regex r("(\\d+),(\\d+)");

  const boost::sregex_token_iterator end;
  std::vector<int> v; // This type has nothing to do with the type of objects you will be extracting
  v.push_back(1);
  v.push_back(2);

  for (boost::sregex_token_iterator i(s.begin(), s.end(), r, v); i != end;)
  {
    std::stringstream ssX;
    ssX << (*i).str();
    int x;
    ssX >> x;
    ++i;

    std::stringstream ssY;
    ssY << (*i).str();
    int y;
    ssY >> y;
    ++i;

    poly.push_back(Point(x, y));
  }

  for(size_t i = 0; i < poly.size(); ++i)
  {
    std::cout << "(" << poly[i].X << ", " << poly[i].Y << ")" << std::endl;
  }
  std::cout << std::endl;

  return 0;
}
