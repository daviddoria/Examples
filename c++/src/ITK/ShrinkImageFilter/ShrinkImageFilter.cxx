#include "itkImage.h"
#include "itkImageFileWriter.h"
//#include "itkRescaleIntensityImageFilter.h"
#include "itkShrinkImageFilter.h"

typedef itk::Image<unsigned char, 2>  ImageType;

void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  std::cout << "Original size: " << image->GetLargestPossibleRegion().GetSize() << std::endl;

  typedef itk::ShrinkImageFilter <ImageType, ImageType>
          ShrinkImageFilterType;

  ShrinkImageFilterType::Pointer shrinkFilter
          = ShrinkImageFilterType::New();
  shrinkFilter->SetInput(image);
  shrinkFilter->SetShrinkFactor(0, 2); // shrink the first dimension by a factor of 2 (i.e. 100 gets changed to 50)
  shrinkFilter->SetShrinkFactor(1, 3); // shrink the second dimension by a factor of 3 (i.e. 100 gets changed to 33)
  shrinkFilter->Update();

  std::cout << "New size: " << shrinkFilter->GetOutput()->GetLargestPossibleRegion().GetSize() << std::endl;


  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  // Create an image with 2 connected components
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region(start, size);
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

  // Make a white square
  for(unsigned int r = 20; r < 80; r++)
    {
    for(unsigned int c = 20; c < 30; c++)
      {
      ImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      image->SetPixel(pixelIndex, 255);
      }
    }
}
