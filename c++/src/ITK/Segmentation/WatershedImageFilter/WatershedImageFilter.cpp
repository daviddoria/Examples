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
#include "itkGradientMagnitudeImageFilter.h"

// Run with:
// ./WatershedImageFilter threshold level
// e.g.
// ./WatershedImageFilter 0.005 .5
// (A rule of thumb is to set the Threshold to be about 1 / 100 of the Level.)

typedef itk::Image<unsigned char, 2> UnsignedCharImageType;
typedef itk::Image<float, 2> FloatImageType;
typedef itk::RGBPixel<unsigned char>   RGBPixelType;
typedef itk::Image<RGBPixelType, 2>    RGBImageType;
typedef itk::Image<unsigned long, 2>   LabeledImageType;

static void CreateImage(UnsignedCharImageType::Pointer image);
static void PerformSegmentation(FloatImageType::Pointer image, const float threshold, const float level);

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
  
  UnsignedCharImageType::Pointer image = UnsignedCharImageType::New();
  CreateImage(image);
  
  typedef itk::GradientMagnitudeImageFilter<
    UnsignedCharImageType, FloatImageType >  GradientMagnitudeImageFilterType;
  GradientMagnitudeImageFilterType::Pointer gradientMagnitudeImageFilter = GradientMagnitudeImageFilterType::New();
  gradientMagnitudeImageFilter->SetInput(image);
  gradientMagnitudeImageFilter->Update();

  // Custom parameters
  PerformSegmentation(gradientMagnitudeImageFilter->GetOutput(), threshold, level);
  
  // Fixed parameters
  PerformSegmentation(gradientMagnitudeImageFilter->GetOutput(), .0025, .25);
  PerformSegmentation(gradientMagnitudeImageFilter->GetOutput(), .005, .5);
  PerformSegmentation(gradientMagnitudeImageFilter->GetOutput(), .0075, .75);
  PerformSegmentation(gradientMagnitudeImageFilter->GetOutput(), .009, .9);
  
  return EXIT_SUCCESS;
}


void CreateImage(UnsignedCharImageType::Pointer image)
{
  // Create a white image with 3 dark regions of different values
  
  itk::Index<2> start;
  start.Fill(0);

  itk::Size<2> size;
  size.Fill(200);

  itk::ImageRegion<2> region(start,size);
  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(255);

  itk::ImageRegionIterator<UnsignedCharImageType> imageIterator(image,region);
 
  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] > 20 && imageIterator.GetIndex()[0] < 50 &&
       imageIterator.GetIndex()[1] > 20 && imageIterator.GetIndex()[1] < 50)
    imageIterator.Set(50);
 
    ++imageIterator;
    }
    
  imageIterator.GoToBegin();
  
  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] > 60 && imageIterator.GetIndex()[0] < 80 &&
       imageIterator.GetIndex()[1] > 60 && imageIterator.GetIndex()[1] < 80)
    imageIterator.Set(100);
 
    ++imageIterator;
    }
    
  imageIterator.GoToBegin();
  
  while(!imageIterator.IsAtEnd())
    {
    if(imageIterator.GetIndex()[0] > 100 && imageIterator.GetIndex()[0] < 130 &&
       imageIterator.GetIndex()[1] > 100 && imageIterator.GetIndex()[1] < 130)
    imageIterator.Set(150);
 
    ++imageIterator;
    }
    
  typedef itk::ImageFileWriter<UnsignedCharImageType> FileWriterType;
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName("input.png");
  writer->SetInput(image);
  writer->Update();
}

void PerformSegmentation(FloatImageType::Pointer image, const float threshold, const float level)
{
  typedef itk::WatershedImageFilter<FloatImageType> WatershedFilterType;
  WatershedFilterType::Pointer watershed = WatershedFilterType::New();
  watershed->SetThreshold(threshold);
  watershed->SetLevel(level);
  watershed->SetInput(image);
  watershed->Update();

  typedef itk::ScalarToRGBColormapImageFilter<LabeledImageType, RGBImageType> RGBFilterType;
  RGBFilterType::Pointer colormapImageFilter = RGBFilterType::New();
  colormapImageFilter->SetInput(watershed->GetOutput());
  colormapImageFilter->SetColormap( RGBFilterType::Jet );
  colormapImageFilter->Update();

  std::stringstream ss;
  ss << "output_" << threshold << "_" << level << ".png";
  
  typedef itk::ImageFileWriter<RGBImageType> FileWriterType;
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName(ss.str());
  writer->SetInput(colormapImageFilter->GetOutput());
  writer->Update();
    
}
