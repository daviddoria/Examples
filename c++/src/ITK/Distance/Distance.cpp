#include "itkPoint.h"

#include <iostream>
#include <string>

int main(int, char *[])
{
  itk::Point<double,3> p0;
  p0[0] = 0.0;
  p0[1] = 0.0;
  p0[2] = 0.0;

  itk::Point<double,3> p1;
  p1[0] = 1.0;
  p1[1] = 1.0;
  p1[2] = 1.0;

  double dist = p0.EuclideanDistanceTo(p1);
  std::cout << "Dist: " << dist << std::endl;

  return 0;
}

