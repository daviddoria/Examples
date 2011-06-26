#include <iostream>


using namespace std;

int main(int argc, char *argv[])
{
  int test = 1;
  switch ( test ) 
  {
  cout << "before cases" << endl;
  case 1 : 
    cout << 1 << endl;
    break;
  case 2 : 
    cout << 2 << endl;
    break;
  default : 
    cout << "default" << endl;

  }

  return 0;
}
