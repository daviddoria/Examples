#include "vtkSmartPointer.h"
#include "vtkMath.h"

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

int main (int argc, char *argv[])
{
  
  int NumberOfSamples = 3;
  int NumberOfVariables = 2;
  double **X = create_matrix<double> (NumberOfSamples, NumberOfVariables);
  X[0][0] = 1; X[0][1] = 2;
  X[1][0] = 2; X[1][1] = 4;
  X[2][0] = 3; X[2][1] = 6;
    
  double **M = create_matrix<double> ( NumberOfVariables, 1 );
    
  vtkMath::SolveHomogeneousLeastSquares(NumberOfSamples, X, NumberOfVariables, M);
  
  std::cout << "Solution is: " << M[0][0] << " " << M[1][0] << std::endl;
  
  free_matrix(X);
  free_matrix(M);
  
  return 0;
}
