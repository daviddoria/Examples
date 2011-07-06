#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkLabelContourImageFilter.h"
#include "itkConnectedComponentImageFilter.h"
#include "itkMersenneTwisterRandomVariateGenerator.h"
#include "itkLabelStatisticsImageFilter.h"

typedef itk::Image<unsigned char, 2>  ImageType;

void CreateImage(ImageType::Pointer image);

int main(int, char *[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  typedef itk::ConnectedComponentImageFilter <ImageType, ImageType >
    ConnectedComponentImageFilterType;
  ConnectedComponentImageFilterType::Pointer connectedComponentImageFilter
    = ConnectedComponentImageFilterType::New ();
  connectedComponentImageFilter->SetInput(image);
  connectedComponentImageFilter->Update();

  itkLabelStatisticsImageFilter
  
  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("output.png");
  writer->SetInput(rescaleFilter->GetOutput());
  writer->Update();

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  // Create an image with 2 connected components
  ImageType::RegionType region;
  ImageType::IndexType start;
  start[0] = 0;
  start[1] = 0;

  ImageType::SizeType size;
  unsigned int NumRows = 200;
  unsigned int NumCols = 300;
  size[0] = NumRows;
  size[1] = NumCols;

  region.SetSize(size);
  region.SetIndex(start);

  image->SetRegions(region);
  image->Allocate();

  // Create a random number generator
  typedef itk::Statistics::MersenneTwisterRandomVariateGenerator GeneratorType;
  GeneratorType::Pointer generator = GeneratorType::New();
 
  generator->Initialize();
  
  // Make a square
  for(unsigned int r = 20; r < 80; r++)
    {
    for(unsigned int c = 30; c < 100; c++)
      {
      ImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      int value = generator->GetIntegerVariate(5); // Get an int between 0 and 5 (inclusive - that is sample from the set {0,1,2,3,4,5})
      image->SetPixel(pixelIndex, value);
      }
  }

  // Make another square
  for(unsigned int r = 100; r < 130; r++)
    {
    for(unsigned int c = 115; c < 160; c++)
      {
      ImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      int value = generator->GetIntegerVariate(50); // Get an int between 0 and 50, inclusive
      image->SetPixel(pixelIndex, value);
      }
  }
}
