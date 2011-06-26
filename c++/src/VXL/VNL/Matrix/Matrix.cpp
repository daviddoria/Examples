#include <iostream>
#include <fstream>

//VNL
#include <vnl/vnl_vector.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_inverse.h>

#include <vnl/vnl_double_3.h>
#include <vnl/vnl_double_2.h>
#include <vnl/vnl_double_3x3.h>
#include <vnl/vnl_double_2x2.h>

#include <vnl/algo/vnl_real_eigensystem.h>

void TestMatrix();
void TestScalarMultiply();
void TestMatrixInverse();
void TestIdentity();
void TestOuterProduct();
void TestCrossProduct();
void TestTranspose();
void TestFixedSizeMatrix();
void WriteReadMatrix();
void VectorMatrixMultiplication();
void MatrixVectorMultiplication();

//////// Helper Declarations ////////
vnl_matrix<double> VectorToMatrix(const vnl_double_3 &A);

int main()
{
  std::cout << "VNL Test" << std::endl;

  //TestMatrix();
  //TestScalarMultiply();

  VectorMatrixMultiplication();
  MatrixVectorMultiplication();

  //TestIdentity();
  //TestOuterProduct();
  //TestTranspose();
  //TestFixedSizeMatrix();

  //WriteReadMatrix();
  return 0;
}

void TestCrossProduct()
{
  vnl_double_3 a(1.0, 2.0, 3.0);
  vnl_double_3 b(1.1, 2.5, 3.6);
  vnl_double_3 c = vnl_cross_3d(a,b);
  std::cout << a << std::endl;
  std::cout << b << std::endl;
  std::cout << c << std::endl;
}

void TestScalarMultiply()
{
  vnl_matrix<double> A(3,3);
  A.set_identity();

  std::cout << "A: " << std::endl << A << std::endl;

  A *= 2;
  std::cout << "2*A: " << std::endl << A << std::endl;

}

void VectorMatrixMultiplication()
{
  vnl_matrix<double> A(3,3);
  A.set_identity();

  vnl_vector<double> v(3);
  v(0) = 1; v(1) = 0; v(2) = 0;

  std::cout << v*A << std::endl;
}

void MatrixVectorMultiplication()
{
  vnl_matrix<double> A(3,3);
  A.set_identity();

  vnl_vector<double> v(2);
  v(0) = 1; v(1) = 0; //v(2) = 0;

  std::cout << A*v << std::endl;
}

void TestMatrix()
{
  vnl_matrix<double> A(3,3);
  std::cout << "Rows: " << A.rows() << std::endl;
  std::cout << "Cols: " << A.columns() << std::endl;
  A(0,0) = 1.0;
  A(0,1) = 2.0;
  A(0,2) = 3.0;
  A(1,0) = 3.0;
  A(1,1) = 2.0;
  A(1,2) = 4.0;
  A(2,0) = 5.0;
  A(2,1) = 6.0;
  A(2,2) = 9.0;
  std::cout << "A: " << A << std::endl;

  vnl_vector< double > p(3);
  p(0) = 2.0;
  p(1) = 4.0;
  p(2) = 6.0;

  std::cout << "p: " << p << std::endl;

  std::cout << "A*p: " << A*p << std::endl;

  //should be (28, 38, 88)
}

void TestMatrixInverse()
{
  vnl_matrix<double> A(3,2);
  std::cout << "Rows: " << A.rows() << std::endl;
  std::cout << "Cols: " << A.columns() << std::endl;
  A(0,0) = 1;
  A(0,1) = 2;
  A(1,0) = 4;
  A(1,1) = 3;
  A(2,0) = 6;
  A(2,1) = 3;
  std::cout << "A = " << A << std::endl;

  vnl_vector< double > b(3);
  b(0) = 4;
  b(1) = 5;
  b(2) = 6;

  std::cout << "b = " << b << std::endl;

  //Ax - b = 0, solution should be x = (.0563, 1.7817)
  vnl_vector<double> x(3);
  //x = vnl_matrix_inverse<double> (A) * b;
  x = vnl_inverse(A) * b;

  std::cout << "x = " << x << std::endl;
}

void TestIdentity()
{
  vnl_matrix<double> A(3,3);
  A.set_identity();
  std::cout << "A = " << A << std::endl;
}

void TestOuterProduct()
{
  vnl_double_3 A(1,2,3);
  vnl_matrix<double> A_mat = VectorToMatrix(A);
  std::cout << "A = " << std::endl << A << std::endl;
  std::cout << "A = " << std::endl << A_mat << std::endl;

  //vnl_double_3 AT = A.transpose();
  vnl_double_3 B(4,5,6);
  vnl_matrix<double> B_mat = VectorToMatrix(B);
  std::cout << "B = " << std::endl << B << std::endl;
  std::cout << "B = " << std::endl << B_mat << std::endl;
  std::cout << "Bt = " << std::endl << B_mat.transpose() << std::endl;

  vnl_matrix<double> OP = A_mat*B_mat.transpose();

  std::cout << "OP = " << std::endl << OP << std::endl;
}

void TestTranspose()
{
  vnl_matrix<double> A(3,2);
  std::cout << "Rows: " << A.rows() << std::endl;
  std::cout << "Cols: " << A.columns() << std::endl;
  A(0,0) = 1;
  A(0,1) = 2;
  A(1,0) = 4;
  A(1,1) = 3;
  A(2,0) = 6;
  A(2,1) = 3;
  std::cout << "A = " << std::endl << A << std::endl;

  std::cout << "A^T = " << std::endl << A.transpose() << std::endl;
}

void TestFixedSizeMatrix()
{
  vnl_double_3x3 A;
  A(0,0) = 1;
  A(0,1) = 2;
  A(0,2) = 3;
  A(1,0) = 4;
  A(1,1) = 5;
  A(1,2) = 6;
  A(2,0) = 7;
  A(2,1) = 8;
  A(2,2) = 9;

  std::cout << "A: " << std::endl << A << std::endl;

  std::cout << "A^T = " << std::endl << A.transpose() << std::endl;
}

void WriteReadMatrix()
{
  vnl_double_3x3 A;
  A(0,0) = 1;
  A(0,1) = 2;
  A(0,2) = 3;
  A(1,0) = 4;
  A(1,1) = 5;
  A(1,2) = 6;
  A(2,0) = 7;
  A(2,1) = 8;
  A(2,2) = 9;

  std::cout << "A: " << std::endl << A << std::endl;
  std::ofstream OutputStream("Test.txt");
  OutputStream << A;
  OutputStream.close();

  std::ifstream InputStream("Test.txt");
  vnl_double_3x3 R; //no member named 'read
  InputStream >> R;
  std::cout << "Input: " << std::endl << R;
}


////////// Helpers /////////////
vnl_matrix<double> VectorToMatrix(const vnl_double_3 &A)
{
  vnl_matrix<double> M(3,1);
  M(0,0) = A(0);
  M(1,0) = A(1);
  M(2,0) = A(2);

  return M;
}