#include <iostream>
#include <set>
#include <vector>
#include <cstdlib>

#include <algorithm>

int main(int argc, char* argv[])
{
  // Create a set
  std::set<unsigned int> s;

  // Add 10 elements to the set
  for(unsigned int i = 0; i < 10; i++)
  {
    s.insert(i);
  }

  std::set<unsigned int>::iterator iterator = s.begin();
  std::advance(iterator, 5);

  std::cout << " " << *iterator;
  return 0;
}

