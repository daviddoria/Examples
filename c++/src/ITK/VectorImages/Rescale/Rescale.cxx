#include "itkVectorImage.h"

// Not currently an implementation to rescale each channel separately.

int main(int, char *[])
{
  // Create an image
  typedef itk::VectorImage<float, 2>  ImageType;

  ImageType::IndexType start;
  start.Fill(0);

  ImageType::SizeType size;
  size.Fill(2);

  ImageType::RegionType region(start,size);

  ImageType::Pointer image = ImageType::New();
  image->SetRegions(region);
  image->SetVectorLength(2);
  image->Allocate();

  return EXIT_SUCCESS;
}