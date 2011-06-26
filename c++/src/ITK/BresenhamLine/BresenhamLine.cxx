#include "itkBresenhamLine.h"
#include "itkVector.h"
#include "itkOffset.h"
#include "itkPoint.h"

#include <iostream>

void Vector();
void Line();

int main(int argc, char *argv[])
{
  Vector();
  Line();

  return EXIT_SUCCESS;
}

void Vector()
{

  itk::BresenhamLine<2> line;

  itk::Vector<float, 2> v;
  v[0] = 1;
  v[1] = 1;
  std::vector< itk::Offset<2> > offsets = line.BuildLine(v, 4);

  for(unsigned int i = 0; i < offsets.size(); i++)
    {
    std::cout << offsets[i] << std::endl;
    }

}

void Line()
{

  itk::BresenhamLine<2> line;
  itk::Index<2> pixel0;
  pixel0[0] = 0;
  pixel0[1] = 0;

  itk::Index<2> pixel1;
  pixel1[0] = 5;
  pixel1[1] = 5;

  std::vector< itk::Index<2> > pixels = line.BuildLine(pixel0, pixel1);

  for(unsigned int i = 0; i < pixels.size(); i++)
    {
    std::cout << pixels[i] << std::endl;
    }

}