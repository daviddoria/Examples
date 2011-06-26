#include <stdio.h>
#include <gsl/gsl_poly.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_blas.h> //basic linear algebra subsystem

#include <iostream>

/* 
MUST INCLUDE THESE LIBRARIES:
libgsl.a libgslcblas.a
*/

using namespace std;

int main (void)
{
	/*
	//solve quadratic equation - #include <gsl/gsl_poly.h>
	double x0=0, y0=0;
	int num_roots;
	num_roots= gsl_poly_solve_quadratic(1,4,3,&x0,&y0);
	cout << x0 << endl << y0 << endl;
	*/

	/*
	//use a matrix - #include <gsl/gsl_matrix.h>
	int i, j;
	gsl_matrix * m = gsl_matrix_alloc (10, 3);
	for (i = 0; i < 10; i++)
		for (j = 0; j < 3; j++)
			gsl_matrix_set (m, i, j, 0.23 + 100*i + j);
	for (i = 0; i < 10; i++)
		for (j = 0; j < 3; j++)
			printf ("m(%d,%d) = %g\n", i, j, gsl_matrix_get (m, i, j));
	*/




	
//	EIGENVALUES/VECTORS - #include <gsl/gsl_eigen.h>
	double data[] = { 1.0 , 1/2.0, 1/3.0, 1/4.0,
					1/2.0, 1/3.0, 1/4.0, 1/5.0,
					1/3.0, 1/4.0, 1/5.0, 1/6.0,
					1/4.0, 1/5.0, 1/6.0, 1/7.0 };
	gsl_matrix_view m = gsl_matrix_view_array (data, 4, 4);
	gsl_vector *eval = gsl_vector_alloc (4);
	gsl_matrix *evec = gsl_matrix_alloc (4, 4);
	gsl_eigen_symmv_workspace * w = gsl_eigen_symmv_alloc (4);
	gsl_eigen_symmv (&m.matrix, eval, evec, w);
	gsl_eigen_symmv_free (w);
	gsl_eigen_symmv_sort (eval, evec, GSL_EIGEN_SORT_ABS_ASC);
	
	
/*
	int i;
	for (i = 0; i < 4; i++)
	{
		double eval_i = gsl_vector_get (eval, i);
		gsl_vector_view evec_i = gsl_matrix_column (evec, i);
		printf ("eigenvalue = %g\n", eval_i);
		printf ("eigenvector = \n");
		gsl_vector_fprintf (stdout,	&evec_i.vector, "%g");
	}
*/
	

	///////////////
	cout << endl << endl;

	gsl_vector *test1 = gsl_vector_alloc(2);
	gsl_vector *test2 = gsl_vector_alloc(2);
	gsl_vector *temp = gsl_vector_alloc(2);
	gsl_vector_set(test1,0,1);
	gsl_vector_set(test1,1,2);
	gsl_vector_memcpy(temp, test1);
	gsl_vector_set(test2,0,3);
	gsl_vector_set(test2,1,4);
	
	cout << test1 << endl << test2 << endl;
	cout << gsl_vector_get(test1,0) << " " << gsl_vector_get(test1,1) << endl;
	cout << gsl_vector_get(test2,0) << " " << gsl_vector_get(test2,1) << endl;
	cout << gsl_vector_get(temp,0) << " " << gsl_vector_get(temp,1) << endl;
	gsl_vector_mul(temp,test2);
	double result=0;
	for (int i = 0; i<2; i++)
	{
		result+=gsl_vector_get(temp,i);
	}
	cout << result << endl;

	gsl_matrix *mat1 = gsl_matrix_alloc(2,1);
	gsl_matrix *mat2 = gsl_matrix_alloc(2,1);
	gsl_matrix *mat3 = gsl_matrix_alloc(2,2);
	gsl_matrix_set(mat1, 0, 0, 1);
	gsl_matrix_set(mat1, 1, 0, 2);
	gsl_matrix_set(mat2, 0, 0, 3);
	gsl_matrix_set(mat2, 1, 0, 4);

	gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1, mat1, mat2, 0, mat3);
	
	cout << gsl_matrix_get(mat3, 0, 0) << " " << gsl_matrix_get(mat3, 0, 1) << endl;
	cout << gsl_matrix_get(mat3, 1, 0) << " " << gsl_matrix_get(mat3, 1, 1) << endl;

	gsl_vector *EigenValues = gsl_vector_alloc (2);
	gsl_matrix *EigenVectors = gsl_matrix_alloc (2,2);

	/*
	gsl_eigen_symmv_workspace * EigenWorkspace = gsl_eigen_symmv_alloc (2);
	gsl_eigen_symmv (mat3, EigenValues, EigenVectors, EigenWorkspace);
	//gsl_eigen_symmv_free (EigenWorkspace);
	//gsl_eigen_symmv_sort (EigenValues, EigenVectors, GSL_EIGEN_SORT_ABS_ASC);

	gsl_matrix_fprintf(stdout, mat3, "%g");

	int i;
	for (i = 0; i < 2; i++)
	{
		double eval_i = gsl_vector_get (EigenValues, i);
		gsl_vector_view evec_i = gsl_matrix_column (EigenVectors, i);
		printf ("eigenvalue = %g\n", eval_i);
		printf ("eigenvector = \n");
		//gsl_vector_fprintf (stdout,	&evec_i.vector, "%g");
		gsl_vector_fprintf (stdout,	&evec_i.vector, "%g");
	}
	*/

	gsl_eigen_hermv_workspace * EigenWorkspace = gsl_eigen_hermv_alloc (2);
	gsl_eigen_hermv (mat3, EigenValues, EigenVectors, EigenWorkspace);
	//gsl_eigen_symmv_free (EigenWorkspace);
	//gsl_eigen_symmv_sort (EigenValues, EigenVectors, GSL_EIGEN_SORT_ABS_ASC);

	gsl_eigen_nonsymmv // in release 1.9 and higher!

	gsl_matrix_fprintf(stdout, mat3, "%g");

	int i;
	for (i = 0; i < 2; i++)
	{
		double eval_i = gsl_vector_get (EigenValues, i);
		gsl_vector_view evec_i = gsl_matrix_column (EigenVectors, i);
		printf ("eigenvalue = %g\n", eval_i);
		printf ("eigenvector = \n");
		//gsl_vector_fprintf (stdout,	&evec_i.vector, "%g");
		gsl_vector_fprintf (stdout,	&evec_i.vector, "%g");
	}

	int MyPause;
	cin >> MyPause;

	return 0;
}