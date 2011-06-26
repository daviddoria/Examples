#include "itkImage.h"
#include "itkSimpleFilterWatcher.h"
#include "itkThresholdImageFilter.h"
#include "itkImageRegionIterator.h"

typedef itk::Image<unsigned char, 2>  ImageType;
static void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  typedef itk::ThresholdImageFilter <ImageType>
          ThresholdImageFilterType;

  ThresholdImageFilterType::Pointer thresholdFilter
          = ThresholdImageFilterType::New();
  thresholdFilter->SetInput(image);
  thresholdFilter->ThresholdBelow(100);
  thresholdFilter->SetOutsideValue(0);

  itk::SimpleFilterWatcher watcher(thresholdFilter, "ThresholdFilter");

  thresholdFilter->Update();

  return EXIT_SUCCESS;
}
void CreateImage(ImageType::Pointer image)
{
  // Create an image with 2 connected components
  ImageType::RegionType region;
  ImageType::IndexType start;
  start[0] = 0;
  start[1] = 0;

  ImageType::SizeType size;
  size[0] = 100;
  size[1] = 100;

  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,region);

  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(255);

    ++imageIterator;
    }

}
