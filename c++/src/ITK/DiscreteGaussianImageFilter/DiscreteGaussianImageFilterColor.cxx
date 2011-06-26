#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkDiscreteGaussianImageFilter.h"
#include "itkCovariantVector.h"

int main(int argc, char * argv[])
{
  typedef itk::Image<itk::CovariantVector< unsigned char, 3>, 2 >   ImageType;
  ImageType::Pointer image = ImageType::New();
  
  typedef itk::DiscreteGaussianImageFilter<
      ImageType, ImageType >  filterType;

  filterType::Pointer gaussianFilter = filterType::New();
  gaussianFilter->SetInput(image);
  gaussianFilter->Update();

  return EXIT_SUCCESS;
}