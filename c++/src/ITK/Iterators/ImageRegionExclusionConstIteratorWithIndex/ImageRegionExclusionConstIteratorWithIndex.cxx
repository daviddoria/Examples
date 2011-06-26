#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageRegionExclusionConstIteratorWithIndex.h"

typedef itk::Image<int, 2>  ImageType;

void CreateImage(ImageType::Pointer);

int main(int argc, char*argv[])
{

  ImageType::SizeType exclusionRegionSize;
  exclusionRegionSize.Fill(1);

  ImageType::IndexType exclusionRegionIndex;
  exclusionRegionIndex.Fill(2);

  ImageType::RegionType exclusionRegion(exclusionRegionIndex, exclusionRegionSize);

  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  itk::ImageRegionExclusionConstIteratorWithIndex<ImageType> imageIterator(image, image->GetLargestPossibleRegion());
  imageIterator.SetExclusionRegion(exclusionRegion);

  unsigned int numberVisited = 0;
  while(!imageIterator.IsAtEnd())
    {
    std::cout << imageIterator.Get() << std::endl;
    ++imageIterator;
    ++numberVisited;
    }

  std::cout << "Visited " << numberVisited << std::endl;

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  ImageType::SizeType regionSize;
  regionSize.Fill(3);

  ImageType::IndexType regionIndex;
  regionIndex.Fill(0);

  ImageType::RegionType region(regionIndex,regionSize);

  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

}