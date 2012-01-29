#include <iostream>
#include <vector>

#include <Eigen/Geometry>
#include <Eigen/Dense> // for Vector
#include <Eigen/StdVector> // Required (http://eigen.tuxfamily.org/dox-devel/TopicStlContainers.html)

// If Eigen complains about "ptrdiff_t does not name a type", #include <cstddef> from the file it is complaining about.

typedef std::vector<Eigen::Vector2d,Eigen::aligned_allocator<Eigen::Vector2d> > Point2DVector;

Point2DVector Generate2DPoints();

int main(int argc, char *argv[])
{
  return 0;
}

Point2DVector Generate2DPoints()
{
  Point2DVector points;

  float x,y;

  x=282;y=274;
  points.push_back(Eigen::Vector2d(x,y));

  x=397;y=227;
  points.push_back(Eigen::Vector2d(x,y));

  x=577;y=271;
  points.push_back(Eigen::Vector2d(x,y));

  x=462;y=318;
  points.push_back(Eigen::Vector2d(x,y));

  x=270;y=479;
  points.push_back(Eigen::Vector2d(x,y));

  x=450;y=523;
  points.push_back(Eigen::Vector2d(x,y));

  x=566;y=475;
  points.push_back(Eigen::Vector2d(x,y));

  return points;
}
