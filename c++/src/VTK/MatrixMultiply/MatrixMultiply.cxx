#include "vtkMath.h"

void OutputMatrix_3x3(double** a);
void OutputMatrix_3x1(double** a);

/* allocate memory for an nrow x ncol matrix */
template<class TReal>
TReal **create_matrix ( long nrow, long ncol )
{
  typedef TReal* TRealPointer;
  TReal **m = new TRealPointer[nrow];

  TReal* block = ( TReal* ) calloc ( nrow*ncol, sizeof ( TReal ) );
  m[0] = block;
  for ( int row = 1; row < nrow; ++row )
  {
    m[ row ] = &block[ row * ncol ];
  }
  return m;
}

/* free a TReal matrix allocated with matrix() */
template<class TReal>
    void free_matrix ( TReal **m )
{
  free ( m[0] );
  delete[] m;
}

template<class TReal>
    void multiply_matrix_3x3_3x1 ( TReal **A, TReal **b, TReal **c )
{
  //store the result of A*b in c
  c[0][0] = A[0][0] * b[0][0] + A[0][1] * b[1][0] + A[0][2] * b[2][0];
  c[1][0] = A[1][0] * b[0][0] + A[1][1] * b[1][0] + A[1][2] * b[2][0];
  c[2][0] = A[2][0] * b[0][0] + A[2][1] * b[1][0] + A[2][2] * b[2][0];
}
	
int main()
{
  double **a = create_matrix<double> ( 3,3 );
  a[0][0] = 1; a[0][1] = 2;  a[0][2] = 3;
  a[1][0] = 4; a[1][1] = 5;  a[1][2] = 6;
  a[2][0] = 7; a[2][1] = 8;  a[2][2] = 9;
  
  double **b = create_matrix<double> ( 3,1 );
  b[0][0] = 1;
  b[1][0] = 1;
  b[2][0] = 1;
  
  double **c = create_matrix<double> ( 3,1 );
  
  multiply_matrix_3x3_3x1(a,b,c);
  
  std::cout << "a: " << std::endl;
  OutputMatrix_3x3(a);
  std::cout << "b: " << std::endl;
  OutputMatrix_3x1(b);
  std::cout << "c: " << std::endl;
  OutputMatrix_3x1(c);
  return 0;
}

void OutputMatrix_3x3(double** a)
{
  std::cout << 
  a[0][0] << " " << a[0][1] << " " << a[0][2] << std::endl
    << a[1][0] << " " << a[1][1] << " " << a[1][2] << std::endl
      << a[2][0] << " " << a[2][1] << " " << a[2][2] << std::endl;
 
}

void OutputMatrix_3x1(double** a)
{
  std::cout << a[0][0] << " " << a[1][0] << " " << a[2][0] << std::endl;
}