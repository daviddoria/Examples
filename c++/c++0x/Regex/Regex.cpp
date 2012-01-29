#include <iostream>
#include <vector>
#include <regex>

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
   std::string s = "Polygon: (1,2), (3,4), (5,6), (7,8)";

   const std::regex r("(\\d+),(\\d+)");

   const std::sregex_token_iterator end;
   std::vector<int> v;
   v.push_back(1);
   v.push_back(2);

   for (std::sregex_token_iterator i(s.begin(), s.end(), r, v);
      i != end;)
   {
      int x = atoi((*i).str().c_str()); ++i;
      int y = atoi((*i).str().c_str()); ++i;
      poly.push_back(Point(x, y));
   }

   for(size_t i = 0; i < poly.size(); ++i)
   {
      std::cout << "(" << poly[i].X << ", " << poly[i].Y << ") ";
   }
   std::cout << std::endl;

   return 0;
}
