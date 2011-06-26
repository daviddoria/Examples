#include <iostream>
#include <set>

#include "Height.h"

int main(int argc, char* argv[])
{
  std::set<Height> S;

  for(unsigned int i = 0; i < 10; i++)
  {
    S.insert(Height(i));
  }

  return 0;
}
