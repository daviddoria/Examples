#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

//using namespace boost::geometry;

#include <iostream>

int main ()
{
  double points[][2] = {{2.0, 1.3}, {4.1, 3.0}, {5.3, 2.6}, {2.9, 0.7}, {2.0, 1.3}};
  boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > poly;
  boost::geometry::append(poly, points);
  //boost::tuple<double, double> p = boost::make_tuple(3.7, 2.0);
  boost::geometry::model::d2::point_xy<double> p(3.7, 2.0);
  std::cout << "Point p is in polygon? " << std::boolalpha << boost::geometry::within(p, poly) << std::endl;

  return 0;
}
