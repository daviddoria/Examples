#include "itkVectorImage.h"
#include "itkNeighborhoodIterator.h"

typedef itk::VectorImage<unsigned char, 2>  VectorImageType;
 
int main(int, char*[])
{
  // Create an image
  VectorImageType::Pointer image = VectorImageType::New();
 
  itk::Index<2> start;
  start.Fill(0);
  
  itk::Size<2> size;
  size.Fill(10);
 
  itk::ImageRegion<2> region(start,size);
 
  image->SetRegions(region);
  image->SetNumberOfComponentsPerPixel(3);
  image->Allocate();
  
  // Create the neighborhood iterator
  VectorImageType::SizeType radius;
  radius[0] = 1;
  radius[1] = 1;
 
  itk::NeighborhoodIterator<VectorImageType> iterator(radius, image, image->GetLargestPossibleRegion());
  
  while(!iterator.IsAtEnd())
    {
    iterator.GetCenterPixel();
 
    ++iterator;
    }
 
 
  return EXIT_SUCCESS;
}
