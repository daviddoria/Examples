#include "vtkMath.h"

void OutputMatrix(double** a);

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
    void transpose_matrix_3x3 ( TReal **m, TReal **mt )
{
  for(unsigned int i = 0; i < 3; i++)
  {
    for(unsigned int j = 0; j < 3; j++)
    {
      mt[j][i] = m[i][j];
    }
  }
}
	
int main()
{
  double **a = create_matrix<double> ( 3,3 );
  a[0][0] = 1; a[0][1] = 2;  a[0][2] = 3;
  a[1][0] = 4; a[1][1] = 5;  a[1][2] = 6;
  a[2][0] = 7; a[2][1] = 8;  a[2][2] = 9;
  
  double **at = create_matrix<double> ( 3,3 );
  //vtkMath::Transpose3x3(a,at);
  transpose_matrix_3x3(a,at);
  
  std::cout << "A: " << std::endl;
  OutputMatrix(a);
  std::cout << "At: " << std::endl;
  OutputMatrix(at);
  return 0;
}

void OutputMatrix(double** a)
{
  std::cout << 
  a[0][0] << " " << a[0][1] << " " << a[0][2] << std::endl
    << a[1][0] << " " << a[1][1] << " " << a[1][2] << std::endl
      << a[2][0] << " " << a[2][1] << " " << a[2][2] << std::endl;
 
}