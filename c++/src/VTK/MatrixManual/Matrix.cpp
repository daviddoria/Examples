#include "vtkSmartPointer.h"
#include "vtkMatrix4x4.h"

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
  double **a = create_matrix<double> ( 3,3 );
  a[0][0] = 1; a[0][1] = 2;  a[0][2] = 3;
  a[1][0] = 2; a[1][1] = 5;  a[1][2] = 9;
  a[2][0] = 3; a[2][1] = 9;  a[2][2] = 8;
  
  return 0;
}
