#include <iostream>
#include <limits>

using namespace std;
		
bool IsNaN(const double a);
		
int main(int argc, char *argv[])
{
  double a = numeric_limits<double>::quiet_NaN();

  double b = 20;
  cout << "Is a NAN? " << a << " " << IsNaN(a) << endl;
  cout << "Is b NAN? " << b << " " << IsNaN(b) << endl;
  return 0;
}

bool IsNaN(const double a)
{
  if(a!=a)
  {
    return true;
  }
  return false;
}
