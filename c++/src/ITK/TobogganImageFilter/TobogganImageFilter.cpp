#include <iostream>

#include "itkCastImageFilter.h"
#include "itkGradientMagnitudeImageFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkScalarToRGBPixelFunctor.h"
#include "itkUnaryFunctorImageFilter.h"
#include "itkVectorCastImageFilter.h"
#include "itkVectorGradientAnisotropicDiffusionImageFilter.h"
#include "itkVectorMagnitudeImageFilter.h"
#include "itkTobogganImageFilter.h"

// Run with:
// input.png output.png 0 .05

int main( int argc, char *argv[] )
{
  // Verify arguments
  if (argc < 3 )
    {
    std::cerr << "Parameters " << std::endl;
    std::cerr << " inputImage outputImage" << std::endl;
    return 1;
    }

  // Parse arguments
  std::string inputFileName = argv[1];
  std::string outputFileName = argv[2];
  
  // Output arguments
  std::cout << "Running with:" << std::endl
            << "Input: " << inputFileName << std::endl
            << "Output: " << outputFileName << std::endl;
  
  typedef itk::Image<float, 2>                       FloatScalarImageType;
  typedef itk::Image<unsigned char, 2>               UnsignedCharScalarImageType;
  
  
  typedef itk::ImageFileReader<FloatScalarImageType> FileReaderType;
  FileReaderType::Pointer reader = FileReaderType::New();
  reader->SetFileName(inputFileName);
  reader->Update();
  
  typedef itk::GradientMagnitudeImageFilter<FloatScalarImageType, FloatScalarImageType> GradientMagnitudeFilterType; 
  GradientMagnitudeFilterType::Pointer gradientMagnitudeFilter = GradientMagnitudeFilterType::New();
  gradientMagnitudeFilter->SetInput(reader->GetOutput());
  gradientMagnitudeFilter->GetOutput();

  typedef itk::TobogganImageFilter<FloatScalarImageType> TobogganFilterType;
  TobogganFilterType::Pointer tobogganFilter = TobogganFilterType::New();
  tobogganFilter->SetInput(gradientMagnitudeFilter->GetOutput());
  tobogganFilter->Update();
    
  /*
  // You could do this, but the result looks black with most viewers because pixels are labeled 0,1,2,3, etc
  typedef itk::Image<itk::IdentifierType, 2>         IdentifierImageType;
  typedef itk::CastImageFilter<IdentifierImageType, UnsignedCharScalarImageType> CastFilterType;
  CastFilterType::Pointer castFilter = CastFilterType::New();
  castFilter->SetInput(tobogganFilter->GetOutput());
  castFilter->Update();
    
  typedef itk::ImageFileWriter<UnsignedCharScalarImageType> FileWriterType;
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName(outputFileName);
  writer->SetInput(castFilter->GetOutput());
  writer->Update();
  */
  
  typedef itk::Functor::ScalarToRGBPixelFunctor<unsigned long> ColorMapFunctorType;
  typedef itk::Image<itk::IdentifierType, 2>         IdentifierImageType;
  typedef itk::RGBPixel<unsigned char>   RGBPixelType;
  typedef itk::Image<RGBPixelType, 2>    RGBImageType;
  
  typedef itk::UnaryFunctorImageFilter<IdentifierImageType,
                                       RGBImageType, ColorMapFunctorType> ColorMapFilterType;
  ColorMapFilterType::Pointer colormapper = ColorMapFilterType::New();
  colormapper->SetInput(tobogganFilter->GetOutput());
  colormapper->Update();
  
  typedef itk::ImageFileWriter<RGBImageType> FileWriterType;
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName(outputFileName);
  writer->SetInput(colormapper->GetOutput());
  writer->Update();
  
  return EXIT_SUCCESS;
}
