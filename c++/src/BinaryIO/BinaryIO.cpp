#include <iostream>
#include <fstream>
#include <cstdlib>

using namespace std;
		
void CWrite();
void CPPWrite();

string Filename = "Test.bin";

int main(int argc, char *argv[])
{
    ofstream fout(Filename.c_str());
    
    fout << 1 << endl << 2 << endl;

    fout.close();

    return 0;
}

void CPPWrite()
{
  float fnum[4] = {11.22, -33.44, 55.66, 77.88};
  int i;

  ofstream out(Filename.c_str(), ios::out | ios::binary);
  if(!out)
  {
	  cout << "Cannot open file.";
	  exit (1);
  }
  
  out.write((char *) &fnum, sizeof(fnum));
  out.close();

}

/*
void CPPRead()
{
  ifstream in("numbers.asc", ios::in | ios::binary);
  if(!in) 
  {
	  cout << "Cannot open file.";
	  exit (1);
  }
  
  in.read((char *) &fnum, sizeof(fnum));
  cout << in.gcount() << " bytes read." << endl;
  for (i=0; i<4; i++)
	  cout << fnum[i] << " ";
  in.close();

}
*/