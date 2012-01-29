#include <gsl/gsl_multimin.h>
#include <iostream>

using namespace std;


double my_f (const gsl_vector *v, void *params)
{
	double x, y;
	double *dp = (double *)params;
	x = gsl_vector_get(v, 0);
	y = gsl_vector_get(v, 1);
	return 10.0 * (x - dp[0]) * (x - dp[0]) +
	20.0 * (y - dp[1]) * (y - dp[1]) + 30.0;
}

int main()
{
	size_t np = 2; //dimension of the problem
	double par[2] = {1.0, 2.0};
	const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex; //nedler-mead simplex algorithm
	gsl_multimin_fminimizer *s = NULL;
	gsl_vector *ss, *x;
	gsl_multimin_function minex_func;
	size_t iter = 0, i;
	int status;
	double size;

	/* Initial vertex size vector */
	ss = gsl_vector_alloc (np);
	
	/* Set all step sizes to 1 */
	gsl_vector_set_all (ss, 1.0);

	/* Starting point */
	x = gsl_vector_alloc (np);
	gsl_vector_set (x, 0, 5.0);
	gsl_vector_set (x, 1, 7.0);

	/* Initialize method and iterate */
	minex_func.f = &my_f;
	minex_func.n = np;
	minex_func.params = (void *)&par;
	s = gsl_multimin_fminimizer_alloc (T, np);
	gsl_multimin_fminimizer_set (s, &minex_func, x, ss);
	do
	{
		iter++;
		status = gsl_multimin_fminimizer_iterate(s);
		if (status)
			break;
		size = gsl_multimin_fminimizer_size (s);
		status = gsl_multimin_test_size (size, 1e-2);

		if (status == GSL_SUCCESS)
		{
			printf ("converged to minimum at\n");
		}

		printf ("%5d ", iter);

		for (i = 0; i < np; i++)
		{
			printf ("%10.3e ", gsl_vector_get (s->x, i));
		}

		printf ("f() = %7.3f size = %.3f\n", s->fval, size);
	}
	while (status == GSL_CONTINUE && iter < 100);

	//clean up
	gsl_vector_free(x);
	gsl_vector_free(ss);
	gsl_multimin_fminimizer_free (s);




	int MyPause;
	cin >> MyPause;

	return 0;
}