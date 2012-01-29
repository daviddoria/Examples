#include <iostream>
#include <sstream>
#include <vector>
#include <boost/regex.hpp>

// This example extracts X and Y from ( X , Y ), (X,Y), (X, Y), etc.


struct Point
{
   float X;
   float Y;
   Point(float x, float y): X(x), Y(y){}
};

typedef std::vector<Point> Polygon;

int main()
{
  Polygon poly;
  std::string s = "Polygon: (1.1,2.2), (3, 4), (5,6), ( 7 , 8.2 )";

  std::string floatRegEx = "(\\s*[0-9]*\\.?[0-9]*\\s*)";

  const boost::regex r("\\(" + floatRegEx + "," + floatRegEx + "\\)");

  const boost::sregex_token_iterator end;
  std::vector<int> v;
  v.push_back(1);
  v.push_back(2);

  for (boost::sregex_token_iterator i(s.begin(), s.end(), r, v); i != end;)
  {
    std::stringstream ssX;
    ssX << (*i).str();
    float x;
    ssX >> x;
    ++i;

    std::stringstream ssY;
    ssY << (*i).str();
    float y;
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
