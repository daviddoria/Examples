#include <itkPoint.h>

#include <iostream>

int main()
{
  typedef itk::Point<double, 2> PointType;

  PointType p0;
  p0[0] = 0.0;
  p0[1] = 0.0;

  std::cout << "p0: " << p0 << std::endl;

  PointType p1;
  p1[0] = 0.0;
  p1[1] = 2.0;

  std::cout << "p1: " << p1 << std::endl;

  std::cout << "Distance: " << p0.EuclideanDistanceTo(p1) << std::endl;
  return 0;
}
