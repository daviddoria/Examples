#include "itkImage.h"
#include "itkNeighborhoodOperatorImageFunction.h"
#include "itkNeighborhoodOperator.h"

typedef itk::Image<unsigned char, 2>  UnsignedCharImageType;

static void CreateImage(UnsignedCharImageType::Pointer image);

int main(int, char*[])
{
  UnsignedCharImageType::Pointer image = UnsignedCharImageType::New();
  CreateImage(image);

  itk::Neighborhood<float, 2> neighborhood;
  neighborhood.SetRadius(1);
  for(unsigned int i = 0; i < neighborhood.GetSize()[0] * neighborhood.GetSize()[1]; ++i)
    {
    neighborhood[i] = 1;
    }

  typedef itk::NeighborhoodOperatorImageFunction<UnsignedCharImageType, float> NeighborhoodOperatorImageFunctionType;
  NeighborhoodOperatorImageFunctionType::Pointer neighborhoodOperatorImageFunction = NeighborhoodOperatorImageFunctionType::New();
  neighborhoodOperatorImageFunction->SetOperator(neighborhood);
  neighborhoodOperatorImageFunction->SetInputImage(image);

  {
  itk::Index<2> index;
  index.Fill(20);

  float output = neighborhoodOperatorImageFunction->EvaluateAtIndex(index);
  std::cout << "Sum on border: " << output << std::endl;
  }
  
  {
  itk::Index<2> index;
  index.Fill(35);

  float output = neighborhoodOperatorImageFunction->EvaluateAtIndex(index);
  std::cout << "Sum in center: " << output << std::endl;
  }
  
  {
  itk::Index<2> index;
  index.Fill(7);

  float output = neighborhoodOperatorImageFunction->EvaluateAtIndex(index);
  std::cout << "Sum outside: " << output << std::endl;
  }
  
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
  image->FillBuffer(0);

  // Make a square
  for(unsigned int r = 20; r < 80; r++)
    {
    for(unsigned int c = 20; c < 80; c++)
      {
      UnsignedCharImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      image->SetPixel(pixelIndex, 1);
      }
    }
}
