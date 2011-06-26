// need to
// export CPLUS_INCLUDE_PATH=$CPLUS_INCLUDE_PATH:/usr/include/octave-3.2.4
// export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/octave-3.2.4
#include <octave/octave.h>
#include <octave/oct.h>
#include <octave/parse.h>

#include <iostream>

void Test1();
void Test2();
void OneParam();
void TestRand();
void Prob();
void SetDir();

int main(int argc, char **argv)
{
  if (octave_main (argc, argv, 1))
  {
    TestRand();
    OneParam();
    Test2();

    SetDir();
    Prob();
  }
  else
  {
    error ("Octave interpreter initialization failed");
  }

  return 0;
}


void Test2()
{
	octave_value_list f_arg, f_ret;

	f_arg(0) = octave_value(2.5);
	f_arg(1) = octave_value(2.6);
	f_ret = feval("min",f_arg);

	Matrix unis (f_ret(0).matrix_value ());
	std::cout << unis;
}


void OneParam()
{
	octave_value_list f_arg, f_ret;
	f_arg(0) = octave_value(-1);
	f_ret = feval("acos", f_arg);
	Matrix unis (f_ret(0).matrix_value ());
	std::cout << unis;
}


void TestRand()
{
	ColumnVector NumRands (2);
	//NumRands(0) = 9000;
	NumRands(0) = 10;
	NumRands(1) = 1;
	
	octave_value_list f_arg;
	f_arg(0) = octave_value (NumRands);
	
	octave_value_list f_ret = feval ("rand", f_arg, 1);
	
	if (f_ret.length () > 0)
	{
		Matrix unis (f_ret(0).matrix_value ());
	
		std::cout << __FILE__ << ":" << __LINE__ << ":" << unis;
	}
}

void Prob()
{
	octave_value_list f_arg, f_ret;

	double c = .01;
	double m = .5;	
	for(double d = 0; d < 5; d++)
	{
		for(int l = 0; l < 5; l++)
		{
		
			f_arg(0) = octave_value(d);
			f_arg(1) = octave_value(l);
			f_arg(2) = octave_value(c);
			f_arg(3) = octave_value(m);
			f_ret = feval("ProbNumeric", f_arg);
			//Matrix unis (f_ret(0).matrix_value ());
			//std::cout << unis;
		
			double P = f_ret(0).double_value();
		
			std::cout << "P: " << P << std::endl;
		}
	}
}

void SetDir()
{
	octave_value_list f_arg, f_ret;
	f_arg(0) = "/media/portable/Examples/c++/src/Octave";
	
	feval("cd", f_arg);
}

