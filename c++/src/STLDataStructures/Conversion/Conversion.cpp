#include <iostream>
#include <set>
#include <vector>
#include <cstdlib>

#include <algorithm>

void SetToVector();

int main(int argc, char* argv[])
{
  SetToVector();
  return 0;
}


void SetToVector()
{
  // Create a set
  std::set<unsigned int> s;

  // Add 10 elements to the set
  for(unsigned int i = 0; i < 10; i++)
  {
    s.insert(i);
  }

  std::cout << "Set contains:" << std::endl;
  // Output all of the elements in the set
  for(std::set<unsigned int>::iterator it1 = s.begin(); it1 != s.end(); it1++)
  {
    std::cout << " " << *it1;
  }

  std::vector<unsigned int> v(s.begin(),s.end());
  std::cout << std::endl << "Vector contains: " << std::endl;
  for(unsigned int i = 0; i < v.size(); i++)
  {
    std::cout << " " << v[i];
  }
}