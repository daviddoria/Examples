// This class has been deprecated in favor of VectorMagnitudeImageFilter
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkGradientMagnitudeImageFilter.h"

int main(int argc, char * argv[])
{
  // Verify command line arguments
  if( argc < 3 )
    {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << "input.png output.png" << std::endl;
    return EXIT_FAILURE;
    }

  // Parse command line arguments
  std::string inputFilename = argv[1];
  std::string outputFilename = argv[2];

  // Setup types
  typedef itk::Image< float,  2 >   FloatImageType;
  typedef itk::Image< unsigned char, 2 >   UnsignedCharImageType;

  typedef itk::GradientMagnitudeImageFilter<
		  UnsignedCharImageType, FloatImageType >  GradientMagnitudeFilterType;

  // Create and setup a reader
  typedef itk::ImageFileReader< UnsignedCharImageType >  ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName( inputFilename.c_str() );

  // Create and setup a gradient filter
  GradientMagnitudeFilterType::Pointer gradientMagnitudeFilter = GradientMagnitudeFilterType::New();
  gradientMagnitudeFilter->SetInput( reader->GetOutput() );
  gradientMagnitudeFilter->Update();

  // To write the gradient image file, we must rescale the gradient values
  // to a reasonable range
  typedef itk::RescaleIntensityImageFilter<
		  FloatImageType, UnsignedCharImageType > RescaleFilterType;

  RescaleFilterType::Pointer rescaleFilter = RescaleFilterType::New();
  rescaleFilter->SetOutputMinimum(0);
  rescaleFilter->SetOutputMaximum(255);
  rescaleFilter->SetInput( gradientMagnitudeFilter->GetOutput() );
  rescaleFilter->Update();
  
  typedef itk::ImageFileWriter< UnsignedCharImageType >  WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName( outputFilename.c_str() );
  writer->SetInput(rescaleFilter->GetOutput());
  writer->Update();
  
  return EXIT_SUCCESS;
}