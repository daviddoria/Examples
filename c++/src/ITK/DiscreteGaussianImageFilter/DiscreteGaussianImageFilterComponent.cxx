#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkDiscreteGaussianImageFilter.h"
#include "itkCovariantVector.h"
#include "itkNthElementImageAdaptor.h"

int main(int argc, char * argv[])
{
  typedef itk::Image<itk::CovariantVector< unsigned char, 3>, 2 >   ColorImageType;
  typedef itk::Image<unsigned char, 2 >   ScalarImageType;
  ColorImageType::Pointer image = ColorImageType::New();

  typedef itk::NthElementImageAdaptor<ColorImageType,
                              unsigned char> ImageAdaptorType;

  ImageAdaptorType::Pointer adaptor = ImageAdaptorType::New();
  adaptor->SelectNthElement(0);
  adaptor->SetImage(image);
  
  typedef itk::DiscreteGaussianImageFilter<
      ImageAdaptorType, ScalarImageType >  filterType;

  filterType::Pointer gaussianFilter = filterType::New();
  gaussianFilter->SetInput(adaptor);
  gaussianFilter->Update();

  return EXIT_SUCCESS;
}