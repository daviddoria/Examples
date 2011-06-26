#include <iostream>
#include <vector>
#include <cstdlib>

#include <cmath>
#include <boost/thread.hpp>
#include <boost/bind.hpp>

using namespace std;

class Test
{
	public:
	void LongFunction();
};

		
int main(int argc, char* argv[])
{
	Test MyTest;
	boost::thread MyThread(boost::bind(&Test::LongFunction, MyTest));
	
	cout << endl << "Stuff in main" << endl;
	
	MyThread.join();
	
	return 0;
}

void Test::LongFunction()
{
	cout << "Start LongFunction" << endl;
	unsigned int BigNum = 1e7;
	
	double temp;
	for(unsigned int i = 0; i < BigNum; i++)
	{
		temp = sin(i) / i;
	}
	cout << "End LongFunction" << endl;
}

