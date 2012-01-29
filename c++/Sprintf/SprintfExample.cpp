#include <iostream>
#include <string>
#include <sstream>

using namespace std;

int main(int argc, char *argv[])
{
	
	string Filenumber;
	
	int num = 43;
	char charnum[5];
	sprintf(charnum, "%05d", num);
	
	
	Filenumber = (string)charnum;
	
	cout << "Num: " << num << " Filenumber: " << Filenumber << endl;
	
	stringstream FileName;
		
	FileName << "Filename_" << Filenumber << ".ext";
	
	cout << "Filename: " << FileName.str();
	
	return 0;
}
