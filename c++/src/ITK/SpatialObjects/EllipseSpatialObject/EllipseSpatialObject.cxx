#include "itkSpatialObjectToImageFilter.h"
#include "itkEllipseSpatialObject.h"
#include "itkImageFileWriter.h"

int main( int argc, char *argv[] )
{
  if( argc != 2 )
    {
    std::cerr << "Usage: " << argv[0] << " outputimagefile " << std::endl;
    return EXIT_FAILURE;
    }

  typedef unsigned char PixelType;
  const unsigned int Dimension = 2;

  typedef itk::Image< PixelType, Dimension >       ImageType;

  typedef itk::EllipseSpatialObject< Dimension >   EllipseType;

  typedef itk::SpatialObjectToImageFilter<
    EllipseType, ImageType >   SpatialObjectToImageFilterType;

  SpatialObjectToImageFilterType::Pointer imageFilter =
    SpatialObjectToImageFilterType::New();

  //  The SpatialObjectToImageFilter requires that the user defines the grid
  //  parameters of the output image. This includes the number of pixels along
  //  each dimension, the pixel spacing, image direction and

  ImageType::SizeType size;
  size[ 0 ] =  50;
  size[ 1 ] =  50;

  imageFilter->SetSize( size );

  ImageType::SpacingType spacing;
  spacing[0] =  100.0 / size[0];
  spacing[1] =  100.0 / size[1];

  imageFilter->SetSpacing( spacing );

  EllipseType::Pointer ellipse    = EllipseType::New();
  EllipseType::ArrayType radiusArray;
  radiusArray[0] = 10;
  radiusArray[1] = 20;
  //ellipse->SetRadius(  size[0] * 0.2 * spacing[0] );
  ellipse->SetRadius(radiusArray);

  // Position the ellipse

  typedef EllipseType::TransformType                 TransformType;

  TransformType::Pointer transform = TransformType::New();

  transform->SetIdentity();

  TransformType::OutputVectorType  translation;
  TransformType::CenterType        center;

  translation[ 0 ] =  size[0] * spacing[0] / 2.0;
  translation[ 1 ] =  size[1] * spacing[1] / 4.0;
  transform->Translate( translation, false );

  ellipse->SetObjectToParentTransform( transform );

  imageFilter->SetInput(ellipse);

  ellipse->SetDefaultInsideValue(255);

  ellipse->SetDefaultOutsideValue(0);

  imageFilter->SetUseObjectValue( true );

  imageFilter->SetOutsideValue( 0 );

  typedef itk::ImageFileWriter< ImageType >     WriterType;
  WriterType::Pointer writer = WriterType::New();

  writer->SetFileName( argv[1] );
  writer->SetInput( imageFilter->GetOutput() );

  try
    {
    imageFilter->Update();
    writer->Update();
    }
  catch( itk::ExceptionObject & excp )
    {
    std::cerr << excp << std::endl;
    return EXIT_FAILURE;
    }

  return EXIT_SUCCESS;
}
