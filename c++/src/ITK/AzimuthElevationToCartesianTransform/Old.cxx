#include "itkPoint.h"
#include "itkAzimuthElevationToCartesianTransform.h"

int main(int, char*[])
{
  typedef itk::Point<double, 3> PointType;
  PointType spherical;
  spherical[0] = 0.0;
  spherical[1] = 45; // set elevation to 45 degrees
  spherical[2] = 1;
  std::cout << "spherical: " << spherical << std::endl;

  typedef itk::AzimuthElevationToCartesianTransform< double, 3 >
    AzimuthElevationToCartesian;
  AzimuthElevationToCartesian::Pointer azimuthElevation =
    AzimuthElevationToCartesian::New();

  std::cout << "Cartesian: " << azimuthElevation->TransformAzElToCartesian(spherical) << std::endl;

  return EXIT_SUCCESS;
}
