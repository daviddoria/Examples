#include <iostream>
#include <csignal>

using namespace std;

void signal_handler(int sig);

int main(int argc, char *argv[])
{
	signal(SIGINT,signal_handler);
	
	cout << "Enter a number: " << endl;
	int a;
	cin >> a;
	
	return 0;
}


/* trap ctrl-c */
void signal_handler(int sig)
{
	cout << "Trapped ctrl-c" << endl;
	//cleanup();
}



