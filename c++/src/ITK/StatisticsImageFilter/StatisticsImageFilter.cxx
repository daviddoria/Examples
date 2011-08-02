#include "itkImage.h"
#include "itkImageFileWriter.h"
#include "itkStatisticsImageFilter.h"
#include "itkImageFileReader.h"

typedef itk::Image<unsigned char, 2>  ImageType;

static void CreateImage(ImageType::Pointer image);

int main(int argc, char*argv[])
{
  ImageType::Pointer image = ImageType::New();
  CreateImage(image);
  
  typedef itk::StatisticsImageFilter<ImageType> StatisticsImageFilterType;
  StatisticsImageFilterType::Pointer statisticsImageFilter
          = StatisticsImageFilterType::New ();
  statisticsImageFilter->SetInput(image);
  statisticsImageFilter->Update();
  
  std::cout << "Mean: " << statisticsImageFilter->GetMean() << std::endl;
  std::cout << "Std.: " << statisticsImageFilter->GetSigma() << std::endl;
  std::cout << "Min: " << statisticsImageFilter->GetMinimum() << std::endl;
  std::cout << "Max: " << statisticsImageFilter->GetMaximum() << std::endl;

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

  // Make a square
  for(unsigned int r = 20; r < 80; r++)
    {
    for(unsigned int c = 30; c < 100; c++)
      {
      ImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      image->SetPixel(pixelIndex, 255);
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

      image->SetPixel(pixelIndex, 255);
      }
  }
}
