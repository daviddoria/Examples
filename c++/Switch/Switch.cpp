#include <iostream>

void Variables();

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

void Variables()
{
  // Can't do this:
//   int test = 1;
//   switch ( test )
//   {
//   case 1 :
//     int a = 0;
//     break;
//   case 2 :
//     cout << 2 << endl;
//     break;
//   }

  // Must use brackets:
  int test = 1;
  switch ( test )
  {
  case 1 :
    {
    int a = 0;
    break;
    }
  case 2 :
    cout << 2 << endl;
    break;
  }

}