#include <iostream>
#include <cmath>

using namespace std;
		
		
int main(int argc, char *argv[])
{
  double a=5.3;

  //cout << a%2; //cant do this
  cout << static_cast<int> (round(a))%2 << endl;
  return 0;
}

