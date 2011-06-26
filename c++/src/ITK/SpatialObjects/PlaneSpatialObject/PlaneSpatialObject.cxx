#include "itkSpatialObjectToImageFilter.h"
#include "itkPlaneSpatialObject.h"
#include "itkImageFileWriter.h"

int main( int argc, char *argv[] )
{
  typedef unsigned char PixelType;
  const unsigned int Dimension = 2;

  typedef itk::Image< PixelType, Dimension >    ImageType;

  typedef itk::PlaneSpatialObject< Dimension >   PlaneType;

  // Create a plane
  PlaneType::Pointer plane = PlaneType::New();

  return EXIT_SUCCESS;
}
