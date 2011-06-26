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
 
int main()
{
  double **A = create_matrix<double> (3,3);
  A[0][0] = 0;
    
  //vtkMath::LUFactorLinearSystem (A, int *index, int size)
  
  return 0;
}
