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
// input.png output.png 0 .05

int main( int argc, char *argv[] )
{
  // Verify arguments
  if (argc < 5 )
    {
    std::cerr << "Parameters " << std::endl;
    std::cerr << " inputImage outputImage threshold level" << std::endl;
    return 1;
    }

  // Parse arguments
  std::string inputFileName = argv[1];
  std::string outputFileName = argv[2];
  
  std::string strThreshold = argv[3];
  float threshold = 0.0;
  std::stringstream ssThreshold;
  ssThreshold << strThreshold;
  ssThreshold >> threshold;
  
  std::string strLevel = argv[4];
  float level = 0.0;
  std::stringstream ssLevel;
  ssLevel << strLevel;
  ssLevel >> level;
  
  // Output arguments
  std::cout << "Running with:" << std::endl
            << "Input: " << inputFileName << std::endl
            << "Output: " << outputFileName << std::endl
            << "Threshold: " << threshold << std::endl
            << "Level: " << level << std::endl;

  typedef itk::RGBPixel<unsigned char>   RGBPixelType;
  typedef itk::Image<RGBPixelType, 2>    RGBImageType;
  typedef itk::Vector<float, 3>          VectorPixelType;
  typedef itk::Image<VectorPixelType, 2> VectorImageType;
  typedef itk::Image<unsigned long, 2>   LabeledImageType;
  typedef itk::Image<float, 2>           ScalarImageType;
  
  typedef itk::ImageFileReader<RGBImageType> FileReaderType;
  FileReaderType::Pointer reader = FileReaderType::New();
  reader->SetFileName(inputFileName);
  reader->Update();

  typedef itk::VectorCastImageFilter<RGBImageType, VectorImageType> CastFilterType;
  CastFilterType::Pointer caster = CastFilterType::New();
  caster->SetInput(reader->GetOutput());
  caster->Update();
  
  typedef itk::VectorGradientAnisotropicDiffusionImageFilter<VectorImageType,
                                                             VectorImageType>  DiffusionFilterType;
  DiffusionFilterType::Pointer diffusion = DiffusionFilterType::New();
  diffusion->SetNumberOfIterations( 10 );
  diffusion->SetConductanceParameter( 2.0 );
  diffusion->SetTimeStep(0.125);
  diffusion->SetInput(caster->GetOutput());
  diffusion->Update();

  typedef itk::VectorMagnitudeImageFilter<VectorImageType, ScalarImageType> VectorMagnitudeFilterType; 
  VectorMagnitudeFilterType::Pointer vectorMagnitudeFilter = VectorMagnitudeFilterType::New();
  //vectorMagnitudeFilter->SetUsePrincipleComponents(atoi(argv[7]));
  vectorMagnitudeFilter->SetInput(diffusion->GetOutput());
  vectorMagnitudeFilter->GetOutput();

  typedef itk::WatershedImageFilter<ScalarImageType> WatershedFilterType;
  WatershedFilterType::Pointer watershed = WatershedFilterType::New();
  watershed->SetLevel( level );
  watershed->SetThreshold( threshold);
  watershed->SetInput(vectorMagnitudeFilter->GetOutput());
  watershed->Update();

  typedef itk::ScalarToRGBColormapImageFilter<LabeledImageType, RGBImageType> RGBFilterType;
  RGBFilterType::Pointer colormapImageFilter = RGBFilterType::New();
  colormapImageFilter->SetInput(watershed->GetOutput());
  colormapImageFilter->SetColormap( RGBFilterType::Hot );
  colormapImageFilter->Update();

  typedef itk::ImageFileWriter<RGBImageType> FileWriterType;
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName(outputFileName);
  writer->SetInput(colormapImageFilter->GetOutput());
  writer->Update();

  return EXIT_SUCCESS;
}
