#include "itkImage.h"
#include "itkImageRegionIterator.h"

template<typename TImage>
static void DeepCopy(typename TImage::Pointer input, typename TImage::Pointer output)
{
  output->SetRegions(input->GetLargestPossibleRegion());
  output->Allocate();

  itk::ImageRegionConstIterator<TImage> inputIterator(input, input->GetLargestPossibleRegion());
  itk::ImageRegionIterator<TImage> outputIterator(output, output->GetLargestPossibleRegion());

  while(!inputIterator.IsAtEnd())
    {
    outputIterator.Set(inputIterator.Get());
    ++inputIterator;
    ++outputIterator;
    }
}

int main(int argc, char *argv[])
{
  typedef itk::Image<unsigned char, 2> ImageType;
  ImageType::Pointer image1 = ImageType::New();
  ImageType::Pointer image2 = ImageType::New();

  DeepCopy<ImageType>(image1, image2);


  return 0;
}
