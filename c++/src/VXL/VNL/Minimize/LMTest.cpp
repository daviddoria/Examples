#include <iostream>

#include <vnl/vnl_least_squares_function.h>
#include <vnl/algo/vnl_levenberg_marquardt.h>

using namespace std;

class DistanceFunction : public vnl_least_squares_function
{
public:
  DistanceFunction(int dof, int num_res, bool with_grad) : vnl_least_squares_function(dof, num_res, with_grad ? use_gradient : no_gradient)
  {
  }

  ~DistanceFunction(){}

  void 	f (vnl_vector< double > const &x, vnl_vector< double > &fx)
  {
    fx(0) = pow(x(0) - 2, 2) + pow(x(1) - 4, 2);
    //fx(0) = pow(x(0) - 2, 2);
    //    fx(1) =  pow(x(1) - 4, 2);
  }

  void trace(int iteration, vnl_vector< double > const &x, vnl_vector< double > const & fx)
  {
 
    cout << "Iteration " << iteration << endl
	 << "---------------" << endl;
    cout << "x: ";
	for (int i = 0; i < x.size(); i++)
	{
		cout << x(i) << " ";
	}

    cout << endl << "f(x): " << fx(0);

  }

};

int main()
{

  int dof = 2;

  DistanceFunction Distance(dof, dof, false);
  vnl_levenberg_marquardt lm(Distance);
  //  lm.set_f_tolerance(1e-3);
  lm.set_max_function_evals(200);
  // lm.set_x_tolerance(1e-4);

  vnl_vector< double > init_params(dof);

  init_params(0) = 10;
  init_params(1) = 10;

  cout << "Initial value is: " << endl
       << init_params(0) << " " << init_params(1) << endl;

  lm.minimize_without_gradient(init_params);
  cout << "Minimum is: " << endl
       << init_params(0) << " " << init_params(1) << endl;

  return 0;
}
