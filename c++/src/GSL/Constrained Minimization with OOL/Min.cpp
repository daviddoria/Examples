//#include <gsl/gsl_multimin.h>
#include <ool/ool_conmin.h>
#include <ool/ool_tools_diff.h> //numerical gradient and hessian
#include <iostream>
#include <math.h>
#include <vector>
#include <gsl/gsl_eigen.h>

using namespace std;
//linked to libgsl.a libgslcblas.a libool.a

void Pause()
{
	int MyPause;
	cin >> MyPause;
}

double my_f (const gsl_vector *v, void *params)
{
	double x, y;
	double *dp = (double *)params;
	x = gsl_vector_get(v, 0);
	y = gsl_vector_get(v, 1);
	double a= pow((1-x),2) + 100*pow(y-pow(x,2),2);
	return a;
}

void my_df (const gsl_vector *v, void *params, gsl_vector *df)
{
	ool_diff_g_auto(&my_f, v, params, df);
}

void my_Hv( const gsl_vector *X, void *params,const gsl_vector *V, gsl_vector *hv )
{
	size_t ii, nn;
	double hvi;

	nn = X->size;

	for( ii = 0; ii < nn; ii++ )
	{
		hvi = 2 * gsl_vector_get( V, ii );
		gsl_vector_set( hv, ii, hvi);
	}
}


/* Compute both f and df together. */
void my_fdf (const gsl_vector *x, void *params, double *f, gsl_vector *df)
{
	*f = my_f(x, params);
	my_df(x, params, df);
}


int main()
{

	//define the number of variables of the problem and the limit for the number of iterations, respectively.
	size_t nn = 2;
	size_t nmax = 10000;
	size_t ii;
	int status;

	 //method dependent. select which optimization algorithm will be used, the SPG algorithm in this case
	const ool_conmin_minimizer_type *T = ool_conmin_minimizer_spg;
	ool_conmin_spg_parameters P;

	//declare variables to hold the objective function, the constraints, the minimizer method, and the initial iterate
	ool_conmin_function   F;
	ool_conmin_constraint C;
	ool_conmin_minimizer *M;
	gsl_vector *X;

	gsl_vector *a;
	a=gsl_vector_alloc(nn);
	gsl_vector_set(a,0,1);

	//The function structure is filled in with the number of variables, pointers to routines to evaluate the objective function and its derivatives, and a pointer to the function parameters.
	F.n   = nn;
	F.f   = &my_f;
	F.df  = &my_df;
	F.fdf = &my_fdf;
	F.Hv  = &my_Hv;
	F.params = (void *) a;

	//The memory allocation to store the bounds. The lower and upper bounds are set to -3 and 3 to all variables.
	C.n = nn;
	C.L = gsl_vector_alloc( C.n );
	C.U = gsl_vector_alloc( C.n );
	gsl_vector_set_all( C.L, -3.0 );
	gsl_vector_set_all( C.U,  3.0 );

	//These two lines allocate and set the initial iterate.
	X = gsl_vector_alloc( nn );
	//gsl_vector_set_all( X, 1.0 );
	gsl_vector_set(X,0,4);
	gsl_vector_set(X,1,4);

	//allocate the necessary memory for an instance of the optimization algorithm of type T. initialize its parameters to default values.
	M = ool_conmin_minimizer_alloc( T, nn );
	ool_conmin_parameters_default( T, (void*)(&P) );

	//everything is put together. It states that this instance of the method M is responsible for minimizing function F, subject to constraints C, starting from point X, with parameters P.
	ool_conmin_minimizer_set( M, &F, &C, X, (void*)(&P) );

	//We are now in position to begin iterating. The iteration counter is initialized and some information concerning the initial point is displayed. The iteration loop is repeated while the maximum number of iterations was not reached and the status is OOL_CONTINUE. Further conditions could also be considered (maximum number of function/gradient evaluation for example). The iteration counter is incremented and one single iteration of the method is performed.  the current iterate is checked for optimality. Finally some information concerning this iteration is displayed.
	ii = 0;
	status = OOL_CONTINUE;

	printf( "%4i : ", ii );
	//iteration_echo ( M );
	gsl_vector *CurrentMinLocation;

	void* temp;
	temp=gsl_vector_alloc(1);

	while( ii < nmax && status == OOL_CONTINUE )
	{
		ii++;
		ool_conmin_minimizer_iterate( M );
		status = ool_conmin_is_optimal( M );

		cout << "Iteration " << ii << endl;
		cout << "Gradient " << M->x << endl;
		cout << "Function Value from M " << ool_conmin_minimizer_minimum(M) << endl;
		CurrentMinLocation = ool_conmin_minimizer_x(M);
		cout << "Function Value from f " << my_f(CurrentMinLocation, temp) << endl;
		
	}

	//The program ends displaying the convergence status, number of variables, function and gradient evaluations, the objective function value at the last iterate and the norm of its projected gradient.
	if(status == OOL_SUCCESS)
		printf("\nConvergence in %i iterations", ii);
	else
		printf("\nStopped with %i iterations", ii);

	printf("\nvariables................: %6i"
	"\nfunction evaluations.....: %6i"
	"\ngradient evaluations.....: %6i"
	"\nfunction value...........: % .6e"
	"\nprojected gradient norm..: % .6e\n",
	nn,
	ool_conmin_minimizer_fcount( M ),
	ool_conmin_minimizer_gcount( M ),
	ool_conmin_minimizer_minimum( M ),
	ool_conmin_minimizer_size( M ));

	cout << "Solution is:" << endl;
	
	//for ( ii = 0; ii < nn; ii++)
	//{
		cout << gsl_vector_get( a, ii) << " " << gsl_vector_get( X, ii) << endl;
	//}

	//To finalize, all allocated memory is freed.
	gsl_vector_free( C.L );
	gsl_vector_free( C.U );
	gsl_vector_free( X );
	gsl_vector_free( a );
	ool_conmin_minimizer_free( M );

	Pause();

	return OOL_SUCCESS;
 
}