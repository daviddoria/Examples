#include "itkImage.h"
#include "itkBinaryThinningImageFilter.h"

//typedef itk::Image<int, 2>  ImageType;
typedef itk::Image<unsigned int, 2>  ImageType;

void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
//  CreateImage(image);

  typedef itk::BinaryThinningImageFilter <ImageType, ImageType>
          BinaryThinningImageFilterType;

  BinaryThinningImageFilterType::Pointer filter
          = BinaryThinningImageFilterType::New();
  filter->SetInput(image);
  filter->Update();

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  // Create an image
  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(100);

  ImageType::RegionType region(start,size);

  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

}
