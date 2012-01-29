#include <iostream>
#include <string>
#include <sstream>

using namespace std;

//test with
//24.5 90.3

int main(int argc, char *argv[])
{
  int NumArgs = argc - 1;
  cout << "Number of arguments: " << NumArgs << endl;
  
  string FirstArgument = argv[1];
  string SecondArgument = argv[2];
  
  cout << "FirstArgument: " << FirstArgument << endl;
  cout << "SecondArgument: " << SecondArgument << endl;
  
  stringstream ssArg1, ssArg2;
  ssArg1 << FirstArgument;
  ssArg2 << SecondArgument;

  double dArg1, dArg2;
  ssArg1 >> dArg1;
  ssArg2 >> dArg2;
  
  cout << "FirstArgument: " << dArg1 << endl;
  cout << "SecondArgument: " << dArg2 << endl;
  
  std::string test = argv[1];
  cout << "argv[1]: " << argv[1] << endl;
  cout << "argv[1] string: " << test << endl;
  
  return 0;
}
