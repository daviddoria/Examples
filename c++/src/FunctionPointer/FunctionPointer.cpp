#include <iostream>

using namespace std;

void add(int a, int b);
void DoThing(void (*pt2Func)(int,int));
		
int main(int argc, char *argv[])
{
  DoThing(add);
  
  return 0;
}

void add(int a, int b)
{
  cout << a+b;
}

void DoThing(void (*pt2Func)(int,int))
{
  pt2Func(2,3);
}