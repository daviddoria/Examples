#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkNeighborhoodIterator.h"

int main(int argc, char*argv[])
{
  // Create an iamge
  itk::Index<2> wholeStart;
  wholeStart.Fill(0);

  itk::Size<2> wholeSize;
  wholeSize.Fill(10);

  itk::ImageRegion<2> wholeRegion(wholeStart, wholeSize);

  typedef itk::Image<unsigned char, 2> ImageType;
  ImageType::Pointer image = ImageType::New();

  image->SetRegions(wholeRegion);

  image->Allocate();
  image->FillBuffer(5);

  // Setup the neighborhood iterator which will be used to iterate over a patch overlapping the boundary of an image
  itk::Size<2> radius;
  radius.Fill(1); // A neighborhood of radius 1 is 3x3

  itk::Size<2> patchSize;
  patchSize.Fill(1);

  itk::Index<2> patchStart;
  //patchStart.Fill(8);
  patchStart.Fill(9);

  itk::ImageRegion<2> region(patchStart, patchSize);

  itk::NeighborhoodIterator<ImageType> iterator(radius, image, region);

  unsigned int iterationCounter = 0;
  while(!iterator.IsAtEnd())
  {
    for(unsigned int i = 0; i < 9; i++) // there are 9 pixels in the neighborhood
      {
      ImageType::IndexType index = iterator.GetIndex(i);
      std::cout << index[0] << " " << index[1] << std::endl;

      bool IsInBounds;
      iterator.GetPixel(i, IsInBounds);
      if(IsInBounds)
        {
        std::cout << "In bounds." << std::endl;
        }
      else
        {
        std::cout << "Out of bounds." << std::endl;
        }
      }
    ++iterator;
    ++iterationCounter;
  }

  std::cout << "There were " << iterationCounter << " iterations." << std::endl; // we expect only 1

  return EXIT_SUCCESS;
}
