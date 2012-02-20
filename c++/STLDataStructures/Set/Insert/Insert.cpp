#include <iostream>
#include <set>
#include <vector>
#include <cstdlib>

#include <algorithm>

int main(int argc, char* argv[])
{
  typedef std::set<unsigned int> SetType;
  SetType s;

  for(unsigned int i = 10; i >= 1; i--)
  {
    s.insert(i);
  }

  // Output all of the elements in the set
  SetType::iterator iter = s.begin();
  s.insert(iter, 1);

  return 0;
}
