#include <iostream>
#include <vector>

using namespace std;

void PercentComplete(int Current, int Total, int Every);

int main(int argc, char *argv[])
{
	int n = 1e5;
	for(int i = 0; i < n; i++)	
	{
		PercentComplete(i, n, 1000);
	}
	return 0;
}

void PercentComplete(int Current, int Total, int Every)
{
	if(Current%Every == 0)
	{
		double Complete = (double)Current / (double)Total;
		cout << Complete << endl;
	}

}