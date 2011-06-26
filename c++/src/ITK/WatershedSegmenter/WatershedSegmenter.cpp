#include <iostream>

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkScalarToRGBPixelFunctor.h"
#include "itkUnaryFunctorImageFilter.h"
#include "itkVectorCastImageFilter.h"
#include "itkVectorGradientAnisotropicDiffusionImageFilter.h"
#include "itkVectorMagnitudeImageFilter.h"
#include "itkWatershedSegmenter.h"

// Run with:
// input.png output.png 0 .05

int main( int argc, char *argv[] )
{
  // Verify arguments
  if (argc < 4 )
    {
    std::cerr << "Parameters " << std::endl;
    std::cerr << " inputImage outputImage threshold" << std::endl;
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
  
  // Output arguments
  std::cout << "Running with:" << std::endl
            << "Input: " << inputFileName << std::endl
            << "Output: " << outputFileName << std::endl
            << "Threshold: " << threshold << std::endl;

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

  typedef itk::watershed::Segmenter<ScalarImageType> WatershedSegmenterType;
  WatershedSegmenterType::Pointer watershedSegmenter = WatershedSegmenterType::New();
  watershedSegmenter->SetThreshold( threshold);
  watershedSegmenter->SetInputImage(vectorMagnitudeFilter->GetOutput());
  watershedSegmenter->Update();
  
  typedef itk::Functor::ScalarToRGBPixelFunctor<unsigned long> ColorMapFunctorType;
  typedef itk::UnaryFunctorImageFilter<LabeledImageType,
                                       RGBImageType, ColorMapFunctorType> ColorMapFilterType;
  ColorMapFilterType::Pointer colormapper = ColorMapFilterType::New();
  colormapper->SetInput(watershedSegmenter->GetOutputImage());
  colormapper->Update();
  
  typedef itk::ImageFileWriter<RGBImageType> FileWriterType;
  FileWriterType::Pointer writer = FileWriterType::New();
  writer->SetFileName(outputFileName);
  writer->SetInput(colormapper->GetOutput());
  writer->Update();

  return EXIT_SUCCESS;
}
