/*
from command line, run with
gcc openmp.cpp -fopenmp -lgomp -lstdc++ -o open
OR
g++ openmp.cpp -fopenmp -lgomp -o open
g++ -fopenmp -lgomp openmp.cpp -o open

with CMake:
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -DGRAPHICS") #g++
*/

#include <omp.h>
#include <iostream>
#include <vector>

void NotParallel();
void Parallel();
void Atomic();

int main (int argc, char *argv[]) 
{
  //NotParallel();
  //Parallel();
  Atomic();
  return 0;
}


void NotParallel()
{
	std::cout << "Not Parallel: " << std::endl;
	int n = 1e8;
	std::vector<int> v(n);
	
	for (int i = 0; i < n; i++)
	{
		v[i] = i;
		//cout << i << endl;
	}
	/*
	for (int i = 0; i < n; i ++)
	{
		cout << v[i];
	}
	*/
	
	std::cout << std::endl;
}


void Parallel()
{
	std::cout << "Parallel: " << std::endl;
		
	/* Get environment information */
	std::cout << "NumProcessors: " << omp_get_num_procs() << std::endl;
	/*
	maxt = omp_get_max_threads();
	inpar = omp_in_parallel();
	dynamic = omp_get_dynamic();
	nested = omp_get_nested();
*/
	std::cout << "NumThreads: " << omp_get_num_threads() << std::endl;
//#pragma omp parallel private(a,c) shared(b) num_threads(2)
	//http://bisqwit.iki.fi/story/howto/openmp/#PrivateFirstprivateAndSharedClauses

	#pragma omp parallel for
	for (int i = 0; i < 10; i++)
	{
		//std::cout << i << std::endl;
		//std::cout << "NumThreads: " << omp_get_num_threads() << std::endl;
		std::cout << omp_get_thread_num() << std::endl;
	}
	
	/*
	for (int i = 0; i < n; i ++)
	{
	std::cout << v[i];
}
	*/
	std::cout << std::endl;

}


void Atomic()
{
	std::cout << "Atomic test: " << std::endl;

	unsigned int counter = 0;
	unsigned int ShouldBe = 1e5;
#pragma omp parallel for
	for (int i = 0; i < ShouldBe; i++)
	{
		counter++;
	}
	
	std::cout << "Counter: " << counter << " (should be " << ShouldBe << ")" << std::endl;
	
	std::cout << std::endl;

}