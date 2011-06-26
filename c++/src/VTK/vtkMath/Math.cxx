#include <vtksys/ios/iostream>

#include "vtkMath.h"
	
void RandomNumbers();
void Pi();
void MultiplyMatrixMatrix();
void MultiplyMatrixMatrixInPlace();

void MultiplyMatrixVector();
void MultiplyMatrixVectorInPlace();

void Transpose();
void TransposeInPlace();

void Invert();
void InvertInPlace();

void OutputVector(double* a);
void OutputMatrix(double a[3][3]);

void Trig();

int main(int argc, char *argv[])
{
  //RandomNumbers();
  //Pi();
  
  
  //MultiplyMatrixVector();
  //MultiplyMatrixMatrix();
  
  
  //MultiplyMatrixVectorInPlace();
  //MultiplyMatrixMatrixInPlace();
  
  //Transpose();
  //TransposeInPlace();

  //Invert();
  //InvertInPlace();
  
  Trig();
  
  return 0;
}

void Trig()
{
  vtkstd::cout << "sin(1) = " << sin(1) << vtkstd::endl;
}

void RandomNumbers()
{
  for(unsigned int i = 0; i < 20; i++)
  {
    std::cout << vtkMath::Gaussian(0.0, 0.2) << std::endl;
  }
}

void Pi()
{

  std::cout << vtkMath::Pi() << vtkstd::endl;
  
}

void MultiplyMatrixMatrix()
{
  
  double a[3][3];
  //row 1
  a[0][0] = 1.0;
  a[0][1] = 0.0;
  a[0][2] = 0.0;
  
  //row 2
  a[1][0] = 4.0;
  a[1][1] = 1.0;
  a[1][2] = 0.0;
  
  //row 3
  a[2][0] = 0.0;
  a[2][1] = 0.0;
  a[2][2] = 1.0;
  
  vtkstd::cout << "a: " << vtkstd::endl;
  OutputMatrix(a);
  
  double b[3][3];
  //row 1
  b[0][0] = 1.0;
  b[0][1] = 0.0;
  b[0][2] = 0.0;
  
  //row 2
  b[1][0] = 0.0;
  b[1][1] = 1.0;
  b[1][2] = 0.0;
  
  //row 3
  b[2][0] = 5.0;
  b[2][1] = 2.0;
  b[2][2] = 1.0;
    
  vtkstd::cout << "b: " << vtkstd::endl;
  OutputMatrix(b);
  
  double result[3][3];
  
  vtkMath::Multiply3x3(a, b, result);
  
  vtkstd::cout << "result: " << result << vtkstd::endl;
  OutputMatrix(result);
  
  //result of A*B should be [1 0 0; 4 1 0; 5 2 1]
}


void MultiplyMatrixMatrixInPlace()
{
  
  double a[3][3];
  //row 1
  a[0][0] = 1.0;
  a[0][1] = 0.0;
  a[0][2] = 0.0;
  
  //row 2
  a[1][0] = 4.0;
  a[1][1] = 1.0;
  a[1][2] = 0.0;
  
  //row 3
  a[2][0] = 0.0;
  a[2][1] = 0.0;
  a[2][2] = 1.0;
  
  vtkstd::cout << "a: " << vtkstd::endl;
  OutputMatrix(a);
  
  double b[3][3];
  //row 1
  b[0][0] = 1.0;
  b[0][1] = 0.0;
  b[0][2] = 0.0;
  
  //row 2
  b[1][0] = 0.0;
  b[1][1] = 1.0;
  b[1][2] = 0.0;
  
  //row 3
  b[2][0] = 5.0;
  b[2][1] = 2.0;
  b[2][2] = 1.0;
    
  vtkstd::cout << "b: " << vtkstd::endl;
  OutputMatrix(b);
  
  vtkMath::Multiply3x3(a, b, a);
  
  vtkstd::cout << "a: " << a << vtkstd::endl;
  OutputMatrix(a);
  
  //result of A*B should be [1 0 0; 4 1 0; 5 2 1]
}


void MultiplyMatrixVector()
{
  
  double a[3][3];
  //row 1
  a[0][0] = 2.0;
  a[0][1] = 0.0;
  a[0][2] = 0.0;
  
  //row 2
  a[1][0] = 0.0;
  a[1][1] = 1.0;
  a[1][2] = 0.0;
  
  //row 3
  a[2][0] = 0.0;
  a[2][1] = 0.0;
  a[2][2] = 1.0;
  
  double v[3] = {1, 2, 3};
  vtkstd::cout << "v: " << v << vtkstd::endl;
  OutputVector(v);
  
  double result[3];
  
  vtkMath::Multiply3x3(a, v, result);
  
  vtkstd::cout << "result: " << result << vtkstd::endl;
  OutputVector(result);
  
  //result should be [2;2;3]
  
}


void MultiplyMatrixVectorInPlace()
{
  
  double A[3][3];
  //row 1
  A[0][0] = 2.0;
  A[0][1] = 0.0;
  A[0][2] = 0.0;
  
  //row 2
  A[1][0] = 0.0;
  A[1][1] = 1.0;
  A[1][2] = 0.0;
  
  //row 3
  A[2][0] = 0.0;
  A[2][1] = 0.0;
  A[2][2] = 1.0;
  
  vtkstd::cout << "A: " << A << vtkstd::endl;
  OutputMatrix(A);
  
  double b[3] = {1, 2, 3};
  vtkstd::cout << "b: " << b << vtkstd::endl;
  OutputVector(b);
  
  vtkMath::Multiply3x3(A, b, b);
  
  vtkstd::cout << "b: " << b << vtkstd::endl;
  OutputVector(b);
  
  //result should be [2;2;3]
  
}

void Transpose()
{
 
  double A[3][3];
  //row 1
  A[0][0] = 1.0;
  A[0][1] = 2.0;
  A[0][2] = 0.0;
  
  //row 2
  A[1][0] = 0.0;
  A[1][1] = 1.0;
  A[1][2] = 0.0;
  
  //row 3
  A[2][0] = 0.0;
  A[2][1] = 0.0;
  A[2][2] = 1.0;
  
  vtkstd::cout << "A: " << A << vtkstd::endl;
  OutputMatrix(A);
  
  double B[3][3];
  
  vtkMath::Transpose3x3(A,B);
  
  vtkstd::cout << "B: " << B << vtkstd::endl;
  OutputMatrix(B);
  
}

void TransposeInPlace()
{

  double A[3][3];
  //row 1
  A[0][0] = 1.0;
  A[0][1] = 2.0;
  A[0][2] = 0.0;
  
  //row 2
  A[1][0] = 0.0;
  A[1][1] = 1.0;
  A[1][2] = 0.0;
  
  //row 3
  A[2][0] = 0.0;
  A[2][1] = 0.0;
  A[2][2] = 1.0;
  
  vtkstd::cout << "A: " << A << vtkstd::endl;
  OutputMatrix(A);
  
  vtkMath::Transpose3x3(A,A);
  
  vtkstd::cout << "A: " << A << vtkstd::endl;
  OutputMatrix(A);
  
}

void Invert()
{

  double A[3][3];
  //row 1
  A[0][0] = 1.0;
  A[0][1] = 2.0;
  A[0][2] = 0.0;
  
  //row 2
  A[1][0] = 0.0;
  A[1][1] = 1.0;
  A[1][2] = 0.0;
  
  //row 3
  A[2][0] = 0.0;
  A[2][1] = 0.0;
  A[2][2] = 1.0;
  
  vtkstd::cout << "A: " << A << vtkstd::endl;
  OutputMatrix(A);
  
  double B[3][3];
  
  vtkMath::Invert3x3(A,B);
  
  vtkstd::cout << "B: " << B << vtkstd::endl;
  OutputMatrix(B);
  
  //result should be [1 -2 0; 0 1 0; 0 0 1]
}

void InvertInPlace()
{

  double A[3][3];
  //row 1
  A[0][0] = 1.0;
  A[0][1] = 2.0;
  A[0][2] = 0.0;
  
  //row 2
  A[1][0] = 0.0;
  A[1][1] = 1.0;
  A[1][2] = 0.0;
  
  //row 3
  A[2][0] = 0.0;
  A[2][1] = 0.0;
  A[2][2] = 1.0;
  
  vtkstd::cout << "A: " << A << vtkstd::endl;
  OutputMatrix(A);
  
  vtkMath::Invert3x3(A,A);
  
  vtkstd::cout << "A: " << A << vtkstd::endl;
  OutputMatrix(A);
  
  //result should be [1 -2 0; 0 1 0; 0 0 1]
}


///////////////////////////

void OutputVector(double* a)
{
  for(unsigned int i = 0; i < 3; i++)
  {
    vtkstd::cout << a[i] << " ";
  }
  
  vtkstd::cout << vtkstd::endl;
}

void OutputMatrix(double a[3][3])
{
  for(unsigned int i = 0; i < 3; i++)
    {
    for(unsigned int j = 0; j < 3; j++)
      {
      vtkstd::cout << a[i][j] << " ";
      }
      vtkstd::cout << vtkstd::endl;
    }
  
  vtkstd::cout << vtkstd::endl;
}