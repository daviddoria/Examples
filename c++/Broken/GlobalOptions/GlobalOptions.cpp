//g++ main.cpp

#include <iostream>
#include <string>

#include "GlobalOptions.h"

void foo();
void bar();

int main(int argc, char **argv)
{
  GlobalOptions::Get().parallel = argv[1];

  foo();
  return 0;
}

void foo()
{
  bar();
}

void bar()
{
  cout << GlobalOptions::Get().parallel << endl;
}
