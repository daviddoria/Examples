#include "itkImage.h"
#include "itkLaplacianImageFilter.h"

int main(int argc, char * argv[])
{
  typedef itk::Image< unsigned char, 2 >   UnsignedCharImageType;
  typedef itk::Image< float,  2 >   FloatImageType;

  typedef itk::LaplacianImageFilter<
          UnsignedCharImageType, FloatImageType >  filterType;
  filterType::Pointer laplacianFilter = filterType::New();
  laplacianFilter->Update();

  return EXIT_SUCCESS;
}