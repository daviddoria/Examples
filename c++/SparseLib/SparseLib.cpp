#define COMPLEX std::complex<double>

#include <iostream>
#include <cstdlib>
#include <comprow_double.h>

using namespace std;

int main() 
{
	cout << "Sparse example" << endl; // prints Sparse example
	
	double val[12]={1,2,3,4,5,6,7,8,9,10,11,12};
	int colind[12]={0,1,4,0,1,2,1,2,4,3,0,4};
	int rowptr[6]={0,3,6,9,10,12};
	
	CompRow_Mat_double R (5,5,12,val,rowptr,colind);
	return 0;
}
