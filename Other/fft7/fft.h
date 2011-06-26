//#include <iostream.h>
//#include <math.h>

typedef complex<double> cmpdbl;
const double pi = 3.141592654;

vector<cmpdbl> fft(vector<cmpdbl> Input) 
{

    double angle;
  cmpdbl W;
  int ke;
   int i,k,j=1;

   cmpdbl Temp;
  cmpdbl U(1,0);
   
	cmpdbl TempComplex;
  vector<cmpdbl> Output;

  //* Determine size of Input data and check that it is power of 2
  int N = Input.size();  // Number of data points
  int M = (int)(log( (double)N )/log(2.0) + 0.5);  // N = 2^M
  int N_Check = (int)(pow(2.0,(double)M) + 0.5); // +.5 is to ensure round to the nearest integer

  if(N != N_Check) 
  {
    cout << "Error in fft(): Number of data points not power of 2" << endl;
    return 0;
  }

  cout << endl << "passed power of 2 test!" << endl;

  //* Bit-scramble the Input data by swapping elements
 
  for( i=1; i<=N-1; i++ )
  {
    if( i < j ) 
	{
		TempComplex=Input[j];
		// Swap elements i and j 
		Input[j]=Input[i];
		Input[i]=TempComplex;
    }

    k = N/2;

    while( k < j )
	{
      j -= k;
      k /= 2;
    }

    j += k;
  }

  cout << endl << "bit scrambled!" << endl;

    	
  //* Loop over number of layers, M = log_2(N)
  for( k=1; k<M; k++ ) 
  {
	cout << endl << "Enter loop 1: " << k << endl;
    ke = (int)(pow(2.0,(double)k) + 0.5);
	
	//* Compute lowest, non-zero power of W for this layer
    
    angle = -pi/(ke/2);

    W=cmpdbl(cos(angle),sin(angle));

	//* Loop over elements in binary order (outer loop)
    for( j=1; j<=ke/2; j++ ) 
	{
		cout << endl << "     Enter loop 2: " << j << endl;
	  //* Loop over elements in binary order (inner loop)
      for( i=j; i<=N; i+=ke ) 
	  {
		  cout << endl << "          Enter loop 3: " << i << endl;
		// Compute the y(.)*W^. factor for this element
		
		Temp=cmpdbl( real(Input[i+ke/2]) * real(U) - imag(Input[i+ke/2]) * imag(U) , real(Input[i+ke/2])*imag(U) + imag(Input[i+ke/2])*real(U));
        
		// Update the current element and its binary pair
        
		Input[i+ke/2]=cmpdbl(real(Input[i]) - real(Temp), imag(Input[i]) - imag(Temp));
        
		Input[i]=cmpdbl(real(Input[i])+real(Temp), imag(Input[i])+imag(Temp));
		
		cout << endl << "          end loop 3" << endl;
      }
	  
	  // Increment the power of W for next set of elements
      U=cmpdbl(((real(U) * real(W)) - (imag(U) * imag(W))), ((real(U)*imag(W)) + (imag(U)*real(W))));

	  cout << endl << "     end loop 2" << endl;
    }
	cout << endl << "end Loop 1" << endl;
  }

for (int h=0; h<2 ; h++)
	{
		cout << endl << Input[h] << endl;
	}

exit(1);
  return Input ;
}
