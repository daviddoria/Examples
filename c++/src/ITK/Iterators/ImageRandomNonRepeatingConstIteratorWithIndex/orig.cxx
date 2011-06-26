#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageRandomNonRepeatingConstIteratorWithIndex.h"

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

  //itk::ImageRandomNonRepeatingConstIteratorWithIndex<ImageType> imageIterator(image, image->GetLargestPossibleRegion());

  itk::ImageRandomNonRepeatingConstIteratorWithIndex<ImageType> imageIterator;
  imageIterator = itk::ImageRandomNonRepeatingConstIteratorWithIndex<ImageType>(image, image->GetLargestPossibleRegion());

  imageIterator.SetNumberOfSamples(image->GetLargestPossibleRegion().GetNumberOfPixels());

  imageIterator.GoToBegin();
  while(!imageIterator.IsAtEnd())
    {
    std::cout << imageIterator.GetIndex() << std::endl;

    ++imageIterator;
    }

  return EXIT_SUCCESS;
}
