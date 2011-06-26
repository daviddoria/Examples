#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageRandomConstIteratorWithIndex.h"

int main(int argc, char*argv[])
{
  typedef itk::Image<unsigned char, 2>  ImageType;
  ImageType::Pointer image = ImageType::New();

  ImageType::SizeType regionSize;
  regionSize.Fill(3);

  ImageType::IndexType regionIndex;
  regionIndex.Fill(0);

  ImageType::RegionType region(regionIndex, regionSize);

  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

  itk::ImageRandomConstIteratorWithIndex<ImageType> imageIterator(image, image->GetLargestPossibleRegion());
  imageIterator.SetNumberOfSamples(region.GetNumberOfPixels());

  imageIterator.GoToBegin();
  while(!imageIterator.IsAtEnd())
    {
    std::cout << imageIterator.GetIndex() << std::endl;

    ++imageIterator;
    }

  return EXIT_SUCCESS;
}

#if 0
void lotsOfSingleSamples()
{
  for(unsigned int i = 0; i < 10; i++)
  {
    imageIterator.GoToBegin();
    while(!imageIterator.IsAtEnd())
      {
      std::cout << imageIterator.GetIndex() << std::endl;

      ++imageIterator;
      }
  }
}

#endif