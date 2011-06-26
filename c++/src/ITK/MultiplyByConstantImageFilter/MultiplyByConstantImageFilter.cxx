#include "itkImage.h"
#include "itkMultiplyByConstantImageFilter.h"
#include "itkImageRegionConstIterator.h"

typedef itk::Image<unsigned char, 2>  ImageType;

void OutputImage(ImageType::Pointer image);

int main( int argc, char *argv[])
{

  ImageType::Pointer image = ImageType::New();

  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(3);

  itk::ImageRegion<2> region(start, size);

  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(2);

  OutputImage(image);

  typedef itk::MultiplyByConstantImageFilter<ImageType, unsigned char, ImageType> MultiplyByConstantImageFilterType;
  MultiplyByConstantImageFilterType::Pointer multiplyByConstantImageFilter = MultiplyByConstantImageFilterType::New();
  multiplyByConstantImageFilter->SetInput(image);
  multiplyByConstantImageFilter->SetConstant(3);
  multiplyByConstantImageFilter->Update();

  OutputImage(multiplyByConstantImageFilter->GetOutput());

  return EXIT_SUCCESS;
}

void OutputImage(ImageType::Pointer image)
{
  itk::ImageRegionConstIterator<ImageType> imageIterator(image,image->GetLargestPossibleRegion());

  while(!imageIterator.IsAtEnd())
    {
    // Get the value of the current pixel
    unsigned char val = imageIterator.Get();
    std::cout << (int)val << std::endl;

    ++imageIterator;
    }

}