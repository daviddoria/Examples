#include "itkCurvatureAnisotropicDiffusionImageFilter.h"
#include "itkGradientMagnitudeRecursiveGaussianImageFilter.h"
#include "itkSigmoidImageFilter.h"
#include "itkFastMarchingImageFilter.h"
#include "itkBinaryThresholdImageFilter.h"
#include "itkImageFileWriter.h"
#include "itkRescaleIntensityImageFilter.h"

typedef itk::Image< unsigned char, 2>  ImageType;

void CreateCircleImage(ImageType::Pointer);

int main( int argc, char *argv[] )
{
  ImageType::Pointer image = ImageType::New();
  CreateCircleImage(image);

  typedef  itk::ImageFileWriter< ImageType  > WriterType;
  WriterType::Pointer writer = WriterType::New();
  writer->SetFileName("circle.png");
  writer->SetInput(image);
  writer->Update();

  typedef itk::BinaryThresholdImageFilter< InternalImageType,
                        OutputImageType    >    ThresholdingFilterType;
  ThresholdingFilterType::Pointer thresholder = ThresholdingFilterType::New();

  thresholder->SetLowerThreshold(           0.0  );
  thresholder->SetUpperThreshold( timeThreshold  );

  thresholder->SetOutsideValue(  0  );
  thresholder->SetInsideValue(  255 );

  typedef  itk::ImageFileReader< InternalImageType > ReaderType;
  typedef  itk::ImageFileWriter<  OutputImageType  > WriterType;

  typedef itk::RescaleIntensityImageFilter<
                               InternalImageType,
                               OutputImageType >   CastFilterType;

  typedef   itk::CurvatureAnisotropicDiffusionImageFilter<
                               InternalImageType,
                               InternalImageType >  SmoothingFilterType;

  SmoothingFilterType::Pointer smoothing = SmoothingFilterType::New();

  typedef   itk::GradientMagnitudeRecursiveGaussianImageFilter<
                               InternalImageType,
                               InternalImageType >  GradientFilterType;
  typedef   itk::SigmoidImageFilter<
                               InternalImageType,
                               InternalImageType >  SigmoidFilterType;

  GradientFilterType::Pointer  gradientMagnitude = GradientFilterType::New();
  SigmoidFilterType::Pointer sigmoid = SigmoidFilterType::New();

  sigmoid->SetOutputMinimum(  0.0  );
  sigmoid->SetOutputMaximum(  1.0  );

  typedef  itk::FastMarchingImageFilter< InternalImageType,
                              InternalImageType >    FastMarchingFilterType;

  FastMarchingFilterType::Pointer  fastMarching = FastMarchingFilterType::New();

  smoothing->SetInput( reader->GetOutput() );
  gradientMagnitude->SetInput( smoothing->GetOutput() );
  sigmoid->SetInput( gradientMagnitude->GetOutput() );
  fastMarching->SetInput( sigmoid->GetOutput() );
  thresholder->SetInput( fastMarching->GetOutput() );
  writer->SetInput( thresholder->GetOutput() );

  smoothing->SetTimeStep( 0.125 );
  smoothing->SetNumberOfIterations(  5 );
  smoothing->SetConductanceParameter( 9.0 );

  gradientMagnitude->SetSigma(  sigma  );

  sigmoid->SetAlpha( alpha );
  sigmoid->SetBeta(  beta  );

  typedef FastMarchingFilterType::NodeContainer           NodeContainer;
  typedef FastMarchingFilterType::NodeType                NodeType;
  NodeContainer::Pointer seeds = NodeContainer::New();

  InternalImageType::IndexType  seedPosition;

  seedPosition[0] = atoi( argv[3] );
  seedPosition[1] = atoi( argv[4] );

  NodeType node;
  const double seedValue = 0.0;

  node.SetValue( seedValue );
  node.SetIndex( seedPosition );

  seeds->Initialize();
  seeds->InsertElement( 0, node );

  fastMarching->SetTrialPoints(  seeds  );

  fastMarching->SetOutputSize(
           reader->GetOutput()->GetBufferedRegion().GetSize() );

  fastMarching->SetStoppingValue(  stoppingTime  );

  return 0;
}

void CreateCircleImage(ImageType::Pointer image)
{
  itk::Size<2> size;
  size.Fill(100);

  itk::Index<2> start;
  start.Fill(0);

  itk::ImageRegion<2> region(start, size);

  image->SetRegions(region);
  image->Allocate();
  image->FillBuffer(0);

  for(unsigned int i = 0; i < 1000; i++)
    {
    int x = 20 * cos(i) + 50;
    int y = 20 * sin(i) + 50;
    itk::Index<2> pixel;
    pixel[0] = x;
    pixel[1] = y;
    image->SetPixel(pixel, 255);
    }
}