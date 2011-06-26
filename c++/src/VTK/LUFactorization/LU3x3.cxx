#include "vtkMath.h"

template<class TReal>
TReal **create_matrix ( long nrow, long ncol ) {
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

void OutputMatrix(double** A)
{

}

int main( int argc, char *argv[] )
{
  //create and populate matrix
  int n = 3;
  double **A = create_matrix<double> (n, n);
  A[0][0] = 1; A[1][0] = 2; A[2][0] = 3;
  A[0][1] = 4; A[1][1] = 5; A[2][1] = 6;
  A[0][2] = 7; A[1][2] = 8; A[2][2] = 0;

  int p[3] = {0, 0, 0};
    
  //Decompose matrix A into LU form. Note: LU decomposition is return in matrix A
  vtkMath::LUFactorLinearSystem(A, p, n);

  //Write to screen results
  std::cout << "L(ower)"<< std::endl;
  std::cout << std::endl;
  std::cout << "[ " << A[0][0] << " - -  " << std::endl;
  std::cout << "  " << A[0][1] << " " << A[1][1] << " -  " << std::endl;
  std::cout << "  " << A[0][2] << " " << A[1][2] << " " << A[2][2] << " ]" << std::endl << std::endl;

  std::cout << "U(pper)"<<std::endl;
  std::cout << std::endl;
  std::cout << "[ "       << A[0][0] << "  "  << A[1][0] << "  " << A[2][0] << std::endl;
  std::cout << "  -  "    << A[1][1] << "  "  << A[2][1] << "  -  " << std::endl;
  std::cout << "  -  -  " << A[2][2] << " ]" << std::endl << std::endl;

  return 0;
}


