#include <iostream>
#include <vector>
#include <cstdlib>

using namespace std;

void TestRangeLoop();
		
int main(int argc, char* argv[])
{
  TestRangeLoop();
  return 0;
}

void TestRangeLoop()
{
  int my_array[5] = {1, 2, 3, 4, 5};
  for(int &x : my_array)
  {
	  x *= 2;
  }
}