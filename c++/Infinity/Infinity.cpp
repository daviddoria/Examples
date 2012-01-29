#include <iostream>
#include <limits>
	
bool IsInf(const double a);
bool IsMinusInf(const double a);
		
int main(int argc, char *argv[])
{

  double a = std::numeric_limits<double>::infinity();
  double minus_inf = -1.0 * std::numeric_limits<double>::infinity();

  std::cout << (1e6 < a) << std::endl;

  double b = 20;
  std::cout << "Is b inf? " << b << " " << IsInf(b) << std::endl;
  std::cout << "Is a inf? " << a << " " << IsInf(a) << std::endl;
  std::cout << "Is a minus infinity? " << a << " " << IsMinusInf(a) << std::endl;
  std::cout << "Is minus_inf infinity? " << minus_inf << " " << IsInf(minus_inf) << std::endl;
  std::cout << "Is minus_inf minus infinity? " << minus_inf << " " << IsMinusInf(minus_inf) << std::endl;
  return 0;
}


bool IsInf(const double a)
{
  if(a == std::numeric_limits<double>::infinity())
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool IsMinusInf(const double a)
{
  if(-1.0*a == std::numeric_limits<double>::infinity())
  {
    return true;
  }
  else
  {
    return false;
  }
}

