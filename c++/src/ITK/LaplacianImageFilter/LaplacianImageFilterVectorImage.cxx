#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkCovariantVector.h"
#include "itkImageFileWriter.h"
#include "itkRescaleIntensityImageFilter.h"
#include "itkLaplacianImageFilter.h"

#include "QuickView.h"

// We would need to decompose the image and take the Laplacian of each channel separately

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
  typedef itk::Image< itk::CovariantVector<float, 3>,  2 >   ImageType;

  typedef itk::ImageFileReader< ImageType >  readerType;
  readerType::Pointer reader = readerType::New();
  reader->SetFileName( inputFilename.c_str() );
  reader->Update();

  typedef itk::LaplacianImageFilter<ImageType, ImageType >  filterType;
  filterType::Pointer laplacianFilter = filterType::New();
  laplacianFilter->SetInput( reader->GetOutput() ); // NOTE: input image type must be double or float
  laplacianFilter->Update();

  QuickView viewer;
  viewer.AddImage<ImageType>(reader->GetOutput());
  viewer.AddImage<ImageType>(laplacianFilter->GetOutput());
  viewer.Visualize();

  return EXIT_SUCCESS;
}