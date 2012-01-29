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
  std::string s = "Polygon: (1.1,2.2), (3, 4), (5,6)";

  std::string floatRegEx = "(\\s*[0-9]*\\.?[0-9]*\\s*)"; // zero or more numerical characters as you want, then an optional '.', then zero or more numerical characters.
  // The \\. is for \. because the first \ is the c++ escpape character and the second \ is the regex escape character

  const boost::regex r("\(" + floatRegEx + "," + floatRegEx + "\)");
  // \s is white space. We want this to allow (2,3) as well as (2, 3) or ( 2 , 3 ) etc.
  // Escapting the ( makes it part of the match. Otherwise it is a regex subgroup indicator.

  const boost::sregex_token_iterator end;
  std::vector<int> v; // This type has nothing to do with the type of objects you will be extracting. These are the indices of the subgroups you are interested in.
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
