#include <iostream>
#include <set>
#include <vector>
#include <cstdlib>

#include <algorithm>
#include <boost/concept_check.hpp>

struct A
{
  int value;
  mutable int additionalValue;
};

bool operator<(const A &a1, const A &a2)
{
  if(a1.value < a2.value)
  {
    return true;
  }

  return false;
}
int main(int argc, char* argv[])
{
  // Create a set
  std::set<A> S;

  // Add 10 elements to the set
  for(unsigned int i = 0; i < 10; i++)
  {
    A a;
    a.value = i;
    a.additionalValue = 2.0;
    S.insert(a);
  }

  // Output all of the elements in the set
  for(std::set<A>::iterator it1 = S.begin(); it1 != S.end(); it1++)
  {
    std::cout << " " << it1->value;
    it1->additionalValue = 3.0;
    std::cout << " " << it1->additionalValue;
  }


  return 0;
}
