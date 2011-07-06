#include <iostream>

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkScalarToRGBPixelFunctor.h"
#include "itkUnaryFunctorImageFilter.h"
#include "itkVectorCastImageFilter.h"
#include "itkVectorGradientAnisotropicDiffusionImageFilter.h"
#include "itkVectorMagnitudeImageFilter.h"
#include "itkWatershedImageFilter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkScalarToRGBColormapImageFilter.h"

// Run with:
// ./WatershedImageFilter threshold level
// e.g.
// ./WatershedImageFilter 0.5 0

typedef itk::Image<unsigned char, 2> ScalarImageType;

void CreateImage(ScalarImageType::Pointer image);

int main( int argc, char *argv[] )
{
  // Verify arguments
  if (argc < 3 )
    {
    std::cerr << "Parameters " << std::endl;
    std::cerr << " threshold level" << std::endl;
    return 1;
    }

  // Parse arguments
  std::string strThreshold = argv[1];
  float threshold = 0.0;
  std::stringstream ssThreshold;
  ssThreshold << strThreshold;
  ssThreshold >> threshold;
  
  std::string strLevel = argv[2];
  float level = 0.0;
  std::stringstream ssLevel;
  ssLevel << strLevel;
  ssLevel >> level;
  
  // Output arguments
  std::cout << "Running with:" << std::endl
            << "Threshold: " << threshold << std::endl
            << "Level: " << level << std::endl;

  typedef itk::RGBPixel<unsigned char>   RGBPixelType;
  typedef itk::Image<RGBPixelType, 2>    RGBImageType;
  typedef itk::Image<unsigned long, 2>   LabeledImageType;
  
  ScalarImageType::Pointer image = ScalarImageType::New();
  CreateImage(image);

  typedef itk::WatershedImageFilter<ScalarImageType> WatershedFilterType;
  WatershedFilterType::Pointer watershed = WatershedFilterType::New();
  watershed->SetLevel(level);
  watershed->SetThreshold(threshold);
  watershed->SetInput(image);
  watershed->Update();

  typedef itk::ScalarToRGBColormapImageFilter<LabeledImageType, RGBImageType> RGBFilterType;
  RGBFilterType::Pointer colormapImageFilter = RGBFilterType::New();
  colormapImageFilter->SetInput(watershed->GetOutput());
  colormapImageFilter->SetColormap( RGBFilterType::Jet );
  colormapImageFilter->Update();

  typedef itk::ImageFileWriter<RGBImageType> FileWriterType;
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName("output.png");
  writer->SetInput(colormapImageFilter->GetOutput());
  writer->Update();

  return EXIT_SUCCESS;
}


void CreateImage(ScalarImageType::Pointer image)
{
  // Create a black image with 2 white regions
  
  ScalarImageType::IndexType start;
  start.Fill(0);

  ScalarImageType::SizeType size;
  size.Fill(200);

  ScalarImageType::RegionType region(start,size);
  image->SetRegions(region);
  image->Allocate();
  //image->FillBuffer(0);
  image->FillBuffer(255);

  // Make a square
  for(unsigned int r = 20; r < 80; r++)
    {
    for(unsigned int c = 30; c < 100; c++)
      {
      ScalarImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      //image->SetPixel(pixelIndex, 255);
      image->SetPixel(pixelIndex, 0);
      }
    }

  // Make another square
  for(unsigned int r = 100; r < 130; r++)
    {
    for(unsigned int c = 115; c < 160; c++)
      {
      ScalarImageType::IndexType pixelIndex;
      pixelIndex[0] = r;
      pixelIndex[1] = c;

      //image->SetPixel(pixelIndex, 255);
      image->SetPixel(pixelIndex, 0);
      }
    }
    
  typedef itk::ImageFileWriter<ScalarImageType> FileWriterType;
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName("input.png");
  writer->SetInput(image);
  writer->Update();
}
