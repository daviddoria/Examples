#include <iostream>
#include <cmath>

#include <vnl/vnl_vector.h>
#include <vnl/vnl_cost_function.h>

#include <vnl/algo/vnl_lbfgs.h> //limited memory BFGS algorithm (general uncontrained optimization)
#include <vnl/algo/vnl_lbfgsb.h>//constrained BFGS
#include <vnl/algo/vnl_amoeba.h>

using namespace std;

void BFGS();
void BFGSB();
void Amoeba();

int main()
{
	//BFGS();
	BFGSB();
	//Amoeba();
	return 0;
}

class DistanceFunction : public vnl_cost_function
{
public:
	DistanceFunction(const int NumVars) : vnl_cost_function(NumVars)
	{
	}
	
	double f(vnl_vector<double> const &x)
	{
		double val = pow(x(0) - 7, 2) + pow(x(1) - 4, 2);
		//double val = pow(x(0), 2) + pow(x(1), 2);
		return val;
	}

	void gradf(vnl_vector<double> const &x, vnl_vector<double> &dx)
	{	
		/*
		//specify manually
		dx(0) = 2*(x(0) - 2);
		dx(1) = 2*(x(1) - 4);
		*/

		//OR, use Finite Difference gradient
		fdgradf(x, dx);
	}

};

void BFGS()
{
	DistanceFunction Distance(2);
	
	vnl_lbfgs Minimizer(Distance);
	Minimizer.set_f_tolerance(1e-6);
	Minimizer.set_trace(true);

	vnl_vector<double> x0(2);
	x0(0) = 10;
	x0(1) = 5;

	cout << "Started at: " << x0 << endl;
	Minimizer.minimize(x0);

	cout << "Ended at: " << x0 << endl;	

	cout << "NumEvals: " << Minimizer.get_num_evaluations() << endl;
 	cout << "NumIterations: " << Minimizer.get_num_iterations() << endl;

}


void BFGSB()
{
	DistanceFunction Distance(2);
	
	vnl_lbfgsb Minimizer(Distance);
	//Minimizer.set_f_tolerance(1e-6);
	vnl_vector<double> l(2);
	l(0) = 7.1;
	l(1) = 4.2;
	vnl_vector<double> u(2);
	u(0) = 10;
	u(1) = 10;
	Minimizer.set_lower_bound(l);
	Minimizer.set_upper_bound(u);
	
	Minimizer.set_trace(true);

	vnl_vector<double> x0(2);
	x0(0) = 9;
	x0(1) = 5;

	cout << "Started at: " << x0 << endl;
	Minimizer.minimize(x0);

	cout << "Ended at: " << x0 << endl;	

	cout << "NumEvals: " << Minimizer.get_num_evaluations() << endl;
	cout << "NumIterations: " << Minimizer.get_num_iterations() << endl;

}

void Amoeba()
{
	DistanceFunction Distance(2);
	
	vnl_amoeba Minimizer(Distance);
	//Minimizer.set_f_tolerance(1e-3);
	Minimizer.set_f_tolerance(1.0);
	Minimizer.set_max_iterations (200);
	
	//you can define the scale of the space ???
	vnl_vector<double> dx(2);
	dx(0) = 1.;
	dx(1) = 1.0;
	
	vnl_vector<double> x0(2);
	x0(0) = 10;
	x0(1) = 5;

	cout << "Started at: " << x0 << endl;
	//Minimizer.minimize(x0);
	Minimizer.minimize(x0, dx);

	cout << "Ended at: " << x0 << endl;	

	cout << "NumEvals: " << Minimizer.get_num_evaluations() << endl;

}