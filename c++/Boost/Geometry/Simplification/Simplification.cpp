#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>

#include <iostream>

int main ()
{
  typedef boost::geometry::model::d2::point_xy<double> xy;

  boost::geometry::model::linestring<xy> line;
  //line += xy(1.1, 1.1), xy(2.5, 2.1), xy(3.1, 3.1), xy(4.9, 1.1), xy(3.1, 1.9);
  line.push_back(xy(1.1, 1.1));
  line.push_back(xy(2.5, 2.1));
  line.push_back(xy(3.1, 3.1));
  line.push_back(xy(4.9, 1.1));
  line.push_back(xy(3.1, 1.9));

  // Simplify it, using distance of 0.5 units
  boost::geometry::model::linestring<xy> simplified;
  boost::geometry::simplify(line, simplified, 0.5);
  /*
  std::cout
      << "  original: " << boost::geometry::dsv(line) << std::endl
      << "simplified: " << boost::geometry::dsv(simplified) << std::endl;
      */
  for(unsigned int i = 0; i < simplified.size(); ++i)
    {
    std::cout << simplified[i].x() << " " << simplified[i].y() << std::endl;
    }  
  return 0;
}
