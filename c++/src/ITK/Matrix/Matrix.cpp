#include <itkMatrix.h>
#include <itkVector.h>

#include <iostream>

void Construct();
void ConstructRunTimeDims();
void Multiply();
void Inverse();

int main()
{
  Construct();
  Multiply();
  return 0;
}

void Construct()
{
  typedef itk::Matrix<double, 3, 3> MatrixType;
  MatrixType M;
  M(0,0) = 1.0;
  M(0,1) = 2.0;
  M(0,2) = 3.0;
  M(1,0) = 4.0;
  M(1,1) = 5.0;
  M(1,2) = 6.0;
  M(2,0) = 7.0;
  M(2,1) = 8.0;
  M(2,2) = 9.0;

  std::cout << "M: " << M << std::endl;
}

void ConstructRunTimeDims()
{
  /*
  int matrixSize = 3;
  typedef itk::Matrix<double, matrixSize, matrixSize> MatrixType;
  MatrixType M;
  M(0,0) = 1.0;
  M(0,1) = 2.0;
  M(0,2) = 3.0;
  M(1,0) = 4.0;
  M(1,1) = 5.0;
  M(1,2) = 6.0;
  M(2,0) = 7.0;
  M(2,1) = 8.0;
  M(2,2) = 9.0;

  std::cout << "M: " << M << std::endl;
  */
}

void Multiply()
{
  typedef itk::Matrix<double, 3, 3> MatrixType;
  MatrixType M;
  M(0,0) = 1.0;
  M(0,1) = 2.0;
  M(0,2) = 3.0;
  M(1,0) = 4.0;
  M(1,1) = 5.0;
  M(1,2) = 6.0;
  M(2,0) = 7.0;
  M(2,1) = 8.0;
  M(2,2) = 9.0;

  std::cout << "M: " << M << std::endl;

  typedef itk::Vector<double, 3> VectorType;
  VectorType V;
  V[0] = 1.0;
  V[1] = 2.0;
  V[2] = 3.0;

  std::cout << "V: " << V << std::endl;

  std::cout << "MV: " << M*V << std::endl;
}

void Inverse()
{

}