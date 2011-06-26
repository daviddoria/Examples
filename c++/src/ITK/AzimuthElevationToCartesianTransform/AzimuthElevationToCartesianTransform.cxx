#include "itkPoint.h"
#include "itkAzimuthElevationToCartesianTransform.h"

void AzElToCartesian();
void BackAndForth();

int main(int, char*[])
{
  AzElToCartesian();

  std::cout << std::endl;

  BackAndForth();

  return EXIT_SUCCESS;
}

void AzElToCartesian()
{
  std::cout << "AzElToCartesian()" << std::endl;

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
  //azimuthElevation->SetMaxAzimuth(360);
  //azimuthElevation->SetMaxElevation(90);

  // These parameters give the correct output
  azimuthElevation->SetMaxAzimuth(1);
  azimuthElevation->SetMaxElevation(1);

  std::cout << "Cartesian: " << azimuthElevation->TransformAzElToCartesian(spherical) << std::endl;
}


void BackAndForth()
{
  std::cout << "BackAndForth()" << std::endl;

  typedef itk::Point<double, 3> PointType;
  PointType point;
  point[0] = 2.458;
  point[1] = .482;
  point[2] = -1.42;
  std::cout << "point: " << point << std::endl;

  typedef itk::AzimuthElevationToCartesianTransform< double, 3 >
    AzimuthElevationToCartesian;
  AzimuthElevationToCartesian::Pointer azimuthElevation =
    AzimuthElevationToCartesian::New();
  //azimuthElevation->SetMaxAzimuth(360);
  //azimuthElevation->SetMaxElevation(90);

  azimuthElevation->SetMaxAzimuth(1);
  azimuthElevation->SetMaxElevation(1);

  PointType azEl = azimuthElevation->TransformCartesianToAzEl(point);
  std::cout << "AzEl: " << azEl << std::endl;

  std::cout << "Converted back: " << azimuthElevation->TransformAzElToCartesian(azEl) << std::endl;

  // The result is the negative of the input (and we would expect it to be identical to the input)
}