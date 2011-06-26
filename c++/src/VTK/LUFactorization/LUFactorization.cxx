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
  std::cout << "[ " << A[0][0] << " " << A[0][1] << std::endl;
  std::cout << "  " << A[1][0] << " " << A[1][1] << " ]" << std::endl;
}

int main( int argc, char *argv[] )
{
  //create and populate matrix
  int n = 2;
  double **A = create_matrix<double> (n, n);
  A[0][0] = 4; A[0][1] = 3;
  A[1][0] = 6; A[1][1] = 3;
  
  //[4 3; 6 3] should decompose to [1 0; 1.5 1] * [4 3; 0 -1.5]
  
  std::cout << "A"<< std::endl;
  OutputMatrix(A);
  
  int p[2] = {0, 0}; //what is p for??
    
  //Decompose matrix A into LU form
  vtkMath::LUFactorLinearSystem(A, p, n);
  
  std::cout << "A decomposed:"<< std::endl;
  OutputMatrix(A);
  
  return 0;
}


