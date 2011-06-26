#include "itkImageRegion.h"
#include "itkIndex.h"
#include "itkSize.h"

int main(int, char *[])
{
  itk::Size<2> size;
  size.Fill(3);

  itk::Index<2> start;
  start.Fill(0);

  typedef itk::ImageRegion<2> RegionType;
  RegionType region(start,size);

  itk::Index<2> testPixel1;
  testPixel1[0] = 1;
  testPixel1[1] = 1;

  itk::Index<2> testPixel2;
  testPixel2[0] = 6;
  testPixel2[1] = 6;

  std::cout << region.IsInside(testPixel1) << std::endl;

  std::cout << region.IsInside(testPixel2) << std::endl;

  return EXIT_SUCCESS;
}
