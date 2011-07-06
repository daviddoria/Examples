#include "itkImage.h"
#include "itkCastImageFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkVectorGradientAnisotropicDiffusionImageFilter.h"

int main( int argc, char *argv[])
{
  // Verify arguments
  if( argc < 3 )
  {
    std::cerr << "Missing Parameters " << std::endl;
    std::cerr << "Usage: " << argv[0];
    std::cerr << " inputImage  outputImage" << std::endl;
    return 1;
  }
  
  // Parse arguments
  std::string inputFileName = argv[1];
  std::string outputFileName = argv[2];

  typedef itk::Image< itk::Vector<float, 3>, 2 >  FloatImageType;
  
  typedef  itk::ImageFileReader< FloatImageType > ReaderType;
  ReaderType::Pointer reader = ReaderType::New();
  reader->SetFileName( inputFileName );
  reader->Update();

  typedef itk::VectorGradientAnisotropicDiffusionImageFilter< FloatImageType, 
               FloatImageType > VectorGradientAnisotropicDiffusionImageFilterType;
  VectorGradientAnisotropicDiffusionImageFilterType::Pointer vectorGradientAnisotropicDiffusionImageFilter = 
      VectorGradientAnisotropicDiffusionImageFilterType::New();
  vectorGradientAnisotropicDiffusionImageFilter->SetInput( reader->GetOutput() );
  vectorGradientAnisotropicDiffusionImageFilter->Update();

  typedef itk::Image< itk::Vector<unsigned char, 3>, 2 >  UnsignedCharImageType;
  typedef itk::CastImageFilter< FloatImageType, UnsignedCharImageType> CastImageFilterType;
  CastImageFilterType::Pointer castImageFilter = CastImageFilterType::New();
  castImageFilter->SetInput( vectorGradientAnisotropicDiffusionImageFilter->GetOutput() );
  castImageFilter->Update();

  typedef  itk::ImageFileWriter< UnsignedCharImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName( outputFileName );
  writer->SetInput( castImageFilter->GetOutput() );
  writer->Update();

  return EXIT_SUCCESS;
}
