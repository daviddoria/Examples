#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkDiscreteGaussianImageFilter.h"
#include "itkSmoothingRecursiveGaussianImageFilter.h"
#include "itkCovariantVector.h"
#include "itkNthElementImageAdaptor.h"

int main(int argc, char * argv[])
{
 const unsigned int Dimension = 2;
 typedef unsigned char PixelComponentType;

 typedef itk::Image<itk::CovariantVector< PixelComponentType, 3>,
           Dimension > ColorImageType;

 typedef itk::Image<PixelComponentType, Dimension >   ScalarImageType;

 ColorImageType::Pointer image = ColorImageType::New();

 typedef itk::NthElementImageAdaptor<ColorImageType,
            PixelComponentType> ImageAdaptorType;

 ImageAdaptorType::Pointer adaptor = ImageAdaptorType::New();
 adaptor->SelectNthElement(0);
 adaptor->SetImage(image);

 typedef itk::SmoothingRecursiveGaussianImageFilter<
    ImageAdaptorType, ScalarImageType >  filterType;

 filterType::Pointer gaussianFilter = filterType::New();
 gaussianFilter->SetInput(adaptor);
 gaussianFilter->Update();

 return EXIT_SUCCESS;
}