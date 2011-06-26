#include "itkMatrix.h"
#include "itkVector.h"
#include "itkCovariantVector.h"

itk::CovariantVector<double, 3> RGBtoCIE(itk::CovariantVector<double,3> rgb);

int main(int, char *[])
{
  itk::CovariantVector<double,3> rgb;
  rgb[0] = 1;
  rgb[1] = 2;
  rgb[2] = 3;


  itk::CovariantVector<double,3> cie = RGBtoCIE(rgb);
  std::cout << "RGB: " << rgb << std::endl
            << "CIE: " << cie << std::endl;

  return EXIT_SUCCESS;
}

itk::CovariantVector<double,3> RGBtoCIE(itk::CovariantVector<double,3> rgb)
{
  typedef itk::Matrix<double, 3, 3> MatrixType;
  MatrixType M;
  M(0,0) = 0.412453 ;
  M(0,1) = 0.357580;
  M(0,2) = 0.180423;
  M(1,0) = 0.212671;
  M(1,1) = 0.715160;
  M(1,2) = 0.072169;
  M(2,0) = 0.019334;
  M(2,1) = 0.119193;
  M(2,2) = 0.950227;

  typedef itk::Vector<double, 3> VectorType;
  VectorType V;
  V[0] = rgb[0];
  V[1] = rgb[1];
  V[2] = rgb[2];

  VectorType XYZ = M*V;

}
/*
octave:2> RGB2Lab(1, 2, 3)
ans =

ans(:,:,1) =  6.5870
ans(:,:,2) = -1.5822
ans(:,:,3) = -5.7994
*/