#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageRegionIterator.h"

int main(int, char*[])
{
  typedef itk::Image<float, 2>  ImageType;

  ImageType::Pointer image = ImageType::New();

  ImageType::SizeType size;
  size[0] = 2;
  size[1] = 2;

  ImageType::IndexType index;
  index[0] = 0;
  index[1] = 0;

  ImageType::RegionType region;
  region.SetSize(size);
  region.SetIndex(index);

  image->SetRegions(region);
  image->Allocate();

  itk::ImageRegionIterator<ImageType> imageIterator(image,image->GetLargestPossibleRegion());

  imageIterator.GoToBegin();
  while(!imageIterator.IsAtEnd())
    {
    imageIterator.Set(0);
    ++imageIterator;
    }

  imageIterator.GoToBegin();
  while(!imageIterator.IsAtEnd())
    {
    std::cout << imageIterator.Get() << std::endl;
    ++imageIterator;
    }

  std::cout << "Round 2: " << std::endl;

  imageIterator.GoToBegin();
  while(!imageIterator.IsAtEnd())
    {
    std::cout << imageIterator.Get() << std::endl;
    ++imageIterator;
    }


  return EXIT_SUCCESS;
}
