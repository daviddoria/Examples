#include "itkImage.h"
#include "itkTestingComparisonImageFilter.h"

int main(int, char*[])
{
  typedef itk::Image<float, 2> ImageType;
  typedef itk::Testing::ComparisonImageFilter< ImageType, ImageType > DiffType;

  
  return EXIT_SUCCESS;
}
