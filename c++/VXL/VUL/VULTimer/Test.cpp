#include <vcl_iostream.h>
#include <vul/vul_timer.h>

#include <cmath>

using namespace std;

void foo();

int main(int argc, char* argv[])
{
	
	vul_timer timer; //create a timer
	//run the function once
	foo();
	//see how long it took
	cout << "Function time: " << timer.real() << " ms" << endl;
	
	timer.mark(); //reset the timer to 0
	//run the function twice
	foo();
	foo();
	//see how long it took
	cout << "Function time: " << timer.real() << " ms" << endl;
	
	return 0;
}

void foo()
{
	double total = 0;
	for(int i = 0; i < 1e6; i ++)
	{
		total += sin(i);
	}
}
