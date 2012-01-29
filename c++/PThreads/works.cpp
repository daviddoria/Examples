#include <iostream>
#include <pthread.h>
#include <cstdlib>

using namespace std;

void *task(void *arg) 
{
	for (;;) 
	{
		cout << (char *)arg;
		cout.flush();
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
	
	task((void *)"2");
	
	return 0;
}
