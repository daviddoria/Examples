#include <iostream>
#include <unordered_set>

// must use -std=c++0x flag

int main(int argc, char* argv[])
{
  // Create a set
  std::unordered_set<unsigned int> S;

  // Add 10 elements to the set
  for(unsigned int i = 0; i < 10; i++)
  {
    S.insert(i);
  }

  // Output all of the elements in the set
  for(std::unordered_set<unsigned int>::iterator it1 = S.begin(); it1 != S.end(); it1++)
  {
    std::cout << " " << *it1;
  }

  return 0;
}
