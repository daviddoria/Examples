#include <iostream>
#include <iomanip>

using namespace std;

int main(int argc, char* argv[])
{
  cout << "d: " << setw(15) << 5.42 << endl;
  cout << setw(4) << "d: " << setw(15) << 5.1 << endl;
  
  return 0;
}
