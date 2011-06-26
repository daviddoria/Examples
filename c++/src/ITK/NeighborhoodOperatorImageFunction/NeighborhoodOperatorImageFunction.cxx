#include "itkImage.h"
#include "itkNeighborhoodOperatorImageFunction.h"
#include "itkNeighborhoodOperator.h"
#include "itkNeighborhoodAllocator.h"
#include "itkNeighborhoodOperatorImageFilter.h"

typedef itk::Image<unsigned char, 2>  UnsignedCharImageType;
typedef itk::Image<float, 2>  FloatImageType;

void CreateImage(UnsignedCharImageType::Pointer image);

int main(int, char*[])
{
  UnsignedCharImageType::Pointer image = UnsignedCharImageType::New();
  CreateImage(image);

  itk::Neighborhood<float, 2> neighborhood;
  neighborhood.SetRadius(1);

  typedef itk::NeighborhoodOperatorImageFilter<UnsignedCharImageType, float> FilterType;
  FilterType::Pointer filter = FilterType::New();
  filter->SetOperator(neighborhood);
  filter->SetInput(image);
  filter->Update();

  itk::Index<2> index;
  index.Fill(10);

  float output = filter->EvaluateAtIndex(index);

  return EXIT_SUCCESS;
}

void CreateImage(UnsignedCharImageType::Pointer image)
{
  // Create an image with 2 connected components
  UnsignedCharImageType::IndexType start;
  start.Fill(0);

  UnsignedCharImageType::SizeType size;
  size.Fill(100);

  UnsignedCharImageType::RegionType region(start,size);

  image->SetRegions(region);
  image->Allocate();

  // Make a square
  for(unsigned int r = 20; r < 80; r++)
    {
    for(unsigned int c = 20; c < 80; c++)
      {
      UnsignedCharImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      image->SetPixel(pixelIndex, 15);
      }
    }
}