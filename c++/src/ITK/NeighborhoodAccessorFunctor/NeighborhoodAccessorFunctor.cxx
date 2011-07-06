#include "itkImage.h"
#include "itkNeighborhoodAccessorFunctor.h"

int main(int, char *[])
{
  typedef itk::Image<float, 2>  ImageType;

  
  ImageType::IndexType start;
  start.Fill(0);
  
  ImageType::SizeType size;
  size.Fill(10);

  ImageType::RegionType region(start,size);
  
  ImageType::Pointer image = ImageType::New();
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

  ImageType::IndexType pixelIndex;
  pixelIndex[0] = 1;
  pixelIndex[1] = 1;

  
  return EXIT_SUCCESS;
}