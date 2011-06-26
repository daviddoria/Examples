#include <iostream>
#include <hash_set> // hash_set is not a member of std

int main(int argc, char* argv[])
{
  // Create a set
  std::hash_set<unsigned int> S;

  // Add 10 elements to the set
  for(unsigned int i = 0; i < 10; i++)
  {
    S.insert(i);
  }

  // Output all of the elements in the set
  for(std::hash_set<unsigned int>::iterator it1 = S.begin(); it1 != S.end(); it1++)
  {
    std::cout << " " << *it1;
  }

  return 0;
}
