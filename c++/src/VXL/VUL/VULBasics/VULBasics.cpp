#include <iostream>
#include <string>

#include <vul/vul_file.h>


using namespace std;

void TestFileExists();

int main(int argc, char **argv)
{
	cout << "VUL Basics" << "--------------" << endl;

	
	TestFileExists();
	
	return 0;
}

void TestFileExists()
{
	string Filename = "/home/doriad/xorg.conf";
	cout << vul_file::exists(Filename) << endl;
	
	Filename = "/home/doriad/xorg.confa";
	cout << vul_file::exists(Filename) << endl;
}