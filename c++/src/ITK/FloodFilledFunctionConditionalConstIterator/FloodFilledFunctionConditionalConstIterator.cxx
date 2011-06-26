#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkRegionGrowImageFilter.h"
#include "itkCastImageFilter.h"

#include "QuickView.h"

typedef itk::Image< unsigned char, 2 >  ImageType;

void CreateImage(ImageType::Pointer image);

int main( int argc, char *argv[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);

  typedef itk::RegionGrowImageFilter<ImageType, ImageType> RegionGrowImageFilterType;
  RegionGrowImageFilterType::Pointer regionGrow = RegionGrowImageFilterType::New();
  regionGrow->SetReplaceValue(255);

  // Seed 1: (25, 35)
  ImageType::IndexType seed1;
  seed1[0] = 25;
  seed1[1] = 35;
  regionGrow->SetSeed(seed1);
  regionGrow->SetInput(image);

  // Seed 2: (110, 120)
  ImageType::IndexType seed2;
  seed2[0] = 110;
  seed2[1] = 120;
  regionGrow->SetSeed(seed2);
  regionGrow->SetReplaceValue(150);

  regionGrow->SetInput(image);


  QuickView viewer;
  viewer.AddImage(reader->GetOutput());
  viewer.AddImage(rescaleFilter->GetOutput());
  viewer.Visualize();

  return EXIT_SUCCESS;
}

void CreateImage(ImageType::Pointer image)
{
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(10);

  itk::ImageRegion region(start,size);
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

  // Make a square
  for(unsigned int r = 3; r < 6; r++)
    {
    for(unsigned int c = 3; c < 6; c++)
      {
      itk::Index<2> pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      image->SetPixel(pixelIndex, 255);
      }
    }
}