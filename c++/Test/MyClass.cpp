#include <iostream>
#include <fstream>
#include <string>

#include "MyClass.h"

using namespace std;


void retrieveSavedata(int stats[4], string filename)
{

	stats[0] = 0;
	stats[1] = 10;
	stats[2] = 20;
	stats[3] = 30;


}

//void outputSavedata (int* stats)
void outputSavedata (int stats[4])
{

	cout<< "\nStrengh:" << stats[0] << "\n";
	cout<< "Defense:" << stats[1] << "\n";
	cout<< "Hitpoints:" << stats[2] << "\n";
	cout<< "Experience:" << stats[3] << "\n";
	

}