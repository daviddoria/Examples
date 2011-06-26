#include "itkImage.h"
#include "itkCastImageFilter.h"
#include "itkCurvatureFlowImageFilter.h"
#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

int main( int argc, char *argv[])
{
  if( argc < 3 )
  {
    std::cerr << "Missing Parameters " << std::endl;
    std::cerr << "Usage: " << argv[0];
    std::cerr << " inputImage  outputImage" << std::endl;
    return 1;
  }

  typedef   float           InternalPixelType;
  typedef unsigned char     ExternalPixelType;
  const     unsigned int    Dimension = 2;
  typedef itk::Image< InternalPixelType, Dimension >  InternalImageType;
  typedef itk::Image< ExternalPixelType, Dimension >  ExternalImageType;

  typedef  itk::ImageFileReader< InternalImageType > ReaderType;
  typedef  itk::ImageFileWriter< ExternalImageType  > WriterType;

  ReaderType::Pointer reader = ReaderType::New();
  WriterType::Pointer writer = WriterType::New();

  reader->SetFileName( argv[1] );
  writer->SetFileName( argv[2] );

  typedef itk::CurvatureFlowImageFilter< InternalImageType, InternalImageType >CurvatureFlowImageFilterType;

  CurvatureFlowImageFilterType::Pointer smoothing = CurvatureFlowImageFilterType::New();

  smoothing->SetInput( reader->GetOutput() );

  typedef itk::CastImageFilter< InternalImageType, ExternalImageType > CastingFilterType;
  CastingFilterType::Pointer caster = CastingFilterType::New();

  caster->SetInput( smoothing->GetOutput() );
  writer->SetInput( caster->GetOutput() );

  smoothing->SetNumberOfIterations( 5 );
  smoothing->SetTimeStep( 0.125 );

  writer->Update();

  return 0;
}