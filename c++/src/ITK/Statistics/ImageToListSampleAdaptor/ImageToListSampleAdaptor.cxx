#include "itkImageToListSampleAdaptor.h"
#include "itkImage.h"
#include "itkRandomImageSource.h"
#include "itkScalarToArrayCastImageFilter.h"

int main()
{

  typedef itk::Image<float,2> FloatImage2DType;

  itk::RandomImageSource<FloatImage2DType>::Pointer random;
  random = itk::RandomImageSource<FloatImage2DType>::New();

  random->SetMin(    0.0 );
  random->SetMax( 1000.0 );

  typedef FloatImage2DType::SpacingValueType  SpacingValueType;
  typedef FloatImage2DType::SizeValueType     SizeValueType;
  typedef FloatImage2DType::PointValueType    PointValueType;

  SizeValueType size[2] = {20, 20};
  random->SetSize( size );

  SpacingValueType spacing[2] = {0.7, 2.1};
  random->SetSpacing( spacing );

  PointValueType origin[2] = {15, 400};
  random->SetOrigin( origin );

  typedef itk::FixedArray< float, 1 > MeasurementVectorType;
  typedef itk::Image< MeasurementVectorType, 2 > ArrayImageType;
  typedef itk::ScalarToArrayCastImageFilter< FloatImage2DType, ArrayImageType >
    CasterType;

  CasterType::Pointer caster = CasterType::New();
  caster->SetInput( random->GetOutput() );
  caster->Update();

  typedef itk::Statistics::ImageToListSampleAdaptor< ArrayImageType > SampleType;
  SampleType::Pointer sample = SampleType::New();

  sample->SetImage( caster->GetOutput() );

  SampleType::Iterator iter = sample->Begin() ;

  while( iter != sample->End() )
    {
    std::cout << iter.GetMeasurementVector() << std::endl ;
    ++iter;
    }

  return EXIT_SUCCESS;
}