#include "itkImage.h"
#include "itkImageRegionConstIterator.h"
#include "itkNormalizeToConstantImageFilter.h"

typedef itk::Image<float, 2>  ImageType;

static void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  // Create an image
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);
  
  typedef itk::NormalizeToConstantImageFilter <ImageType, ImageType> NormalizeToConstantImageFilterType;
  NormalizeToConstantImageFilterType::Pointer normalizeToConstantImageFilter = NormalizeToConstantImageFilterType::New();
  normalizeToConstantImageFilter->SetInput(image);
  normalizeToConstantImageFilter->SetConstant(1);
  normalizeToConstantImageFilter->Update();

  itk::ImageRegionConstIterator<ImageType> imageIterator(normalizeToConstantImageFilter->GetOutput(),
                                                         normalizeToConstantImageFilter->GetOutput()->GetLargestPossibleRegion());
 
  // The output pixels should all be 1/9 (=0.11111)
  while(!imageIterator.IsAtEnd())
    {
    std::cout << imageIterator.Get() << std::endl;
    ++imageIterator;
    }

  return EXIT_SUCCESS;
}

static void CreateImage(ImageType::Pointer image)
{
  // Create an image full of 1's
  
  ImageType::IndexType start;
  start.Fill(0);
 
  ImageType::SizeType size;
  size.Fill(3);
 
  ImageType::RegionType region(start,size);
 
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(1);
  
}
