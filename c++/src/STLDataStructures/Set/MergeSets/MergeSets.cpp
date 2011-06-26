#include <iostream>
#include <set>
#include <algorithm>

void Output(std::set<unsigned int> &S);

int main()
{
  std::set<unsigned int> S1;

  for(unsigned int i = 0; i < 10; i++)
    {
    S1.insert(i);
    }

  std::cout << "S1: " << std::endl;
  Output(S1);

  std::set<unsigned int> S2;

  for(unsigned int i = 11; i < 19; i++)
    {
    S2.insert(i);
    }

  std::cout << "S2: " << std::endl;
  Output(S2);

  S1.insert(S2.begin(), S2.end());

  std::cout << "Combined: " << std::endl;
  Output(S1);

  return 0;
}

void Output(std::set<unsigned int> &S)
{
  for(std::set<unsigned int>::iterator it1 = S.begin(); it1 != S.end(); it1++)
  {
    std::cout << " " << *it1;
  }

  std::cout << std::endl;
}