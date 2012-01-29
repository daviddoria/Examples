#include <iostream>
#include <cstdlib>
#include <cstdio> //for sleep
#include <ctime>

using namespace std;

int main()
{
  int counter;
  for(counter = 1000; counter < 10000; counter++)
  {
    cout << "\r" << counter;
    cout.flush();

    //cout << "hi" << endl;
    //cout << "hi";
    sleep(1);
    
    //cout << counter << endl;
  }
  return 0;
}