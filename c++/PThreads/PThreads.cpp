#include <iostream>
#include <pthread.h>
#include <cstdlib>

using namespace std;

void *task(void *arg) 
{
	for (unsigned int i = 0; i < 1e3; i++) 
	{
		cout << "running..." << endl;
		//cout.flush();
	}
	return NULL;
}

int main() 
{
	pthread_t t1;
	if ( pthread_create(&t1, NULL, task, (void *)"1") != 0 ) 
	{
		cout << "pthread_create() error" << endl;
		abort();
	}
	
	//pthread_join(t1, NULL);
	
	cout << "done." << endl;
	
	return 0;
}
