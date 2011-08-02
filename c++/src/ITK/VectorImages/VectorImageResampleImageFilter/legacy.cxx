#include "itkImage.h"
#include "itkVectorResampleImageFilter.h"
#include "itkVectorImage.h"

int main(int argc, char *argv[])
{
  typedef itk::VectorImage<double, 2> VectorImageType;

  VectorImageType::Pointer image = VectorImageType::New();
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(10);

  itk::ImageRegion<2> region(start,size);
  image->SetRegions(region);
  image->SetNumberOfComponentsPerPixel(3);
  image->Allocate();
  image->FillBuffer(itk::NumericTraits<VectorImageType::InternalPixelType>::Zero);

  typedef itk::VectorResampleImageFilter< VectorImageType, VectorImageType > VectorResampleFilterType;
  VectorResampleFilterType::Pointer vectorResampleFilter = VectorResampleFilterType::New();
  vectorResampleFilter->SetInput(image);
  vectorResampleFilter->Update();

  return EXIT_SUCCESS;
}
