#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkVectorImage.h"
#include "itkVectorMagnitudeImageFilter.h"

int main(int argc, char * argv[])
{
  // Verify command line arguments
  if( argc < 2 )
    {
    std::cerr << "Usage: " << std::endl;
    std::cerr << argv[0] << "inputImageFile" << std::endl;
    return EXIT_FAILURE;
    }

  // Parse command line arguments
  std::string inputFilename = argv[1];

  // Setup types
  typedef itk::VectorImage< float,  2 >    VectorImageType;
  typedef itk::Image< unsigned char, 2 >   UnsignedCharImageType;

  typedef itk::ImageFileReader< VectorImageType >  ReaderType;

  typedef itk::VectorMagnitudeImageFilter<
		  VectorImageType, UnsignedCharImageType >  VectorMagnitudeFilterType;

  // Create and setup a reader
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName( inputFilename.c_str() );

  // Create and setup a gradient filter
  VectorMagnitudeFilterType::Pointer magnitudeFilter = VectorMagnitudeFilterType::New();
  magnitudeFilter->SetInput( reader->GetOutput() );
  magnitudeFilter->Update();

  // To write the magnitude image file, we must rescale the gradient values
  // to a reasonable range
  typedef itk::RescaleIntensityImageFilter<
		  UnsignedCharImageType, UnsignedCharImageType > rescaleFilterType;

  rescaleFilterType::Pointer rescaler = rescaleFilterType::New();
  rescaler->SetOutputMinimum(0);
  rescaler->SetOutputMaximum(255);
  rescaler->SetInput( magnitudeFilter->GetOutput() );
  rescaler->Update();
  
  return EXIT_SUCCESS;
}