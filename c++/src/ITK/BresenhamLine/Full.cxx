#include "itkBresenhamLine.h"
#include "itkVector.h"
#include "itkOffset.h"
#include "itkPoint.h"

#include <iostream>

int main(int argc, char *argv[])
{
  // Get the points on a Bresenham line between (2,2) and (5,5)
  itk::Point<float,2> p0;
  p0[0] = 2;
  p0[1] = 2;

  itk::Point<float,2> p1;
  p1[0] = 5;
  p1[1] = 5;

  float dist = p0.EuclideanDistanceTo(p1);

  itk::BresenhamLine<2> line;

  itk::Vector<float, 2> v;
  v[0] = 1;
  v[1] = 1;
  std::vector< itk::Offset<2> > offsets = line.BuildLine(v, dist);

  itk::Index<2> p0index;
  p0index[0] = p0[0];
  p0index[1] = p0[1];

  for(unsigned int i = 0; i < offsets.size(); i++)
    {
    std::cout << p0index + offsets[i] << std::endl;
    }

  return EXIT_SUCCESS;
}