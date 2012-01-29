#include <iostream>
#include <set>
#include <algorithm>

void Output(std::set<unsigned int> S)
{
  // Output all of the elements in the set
  for(std::set<unsigned int>::iterator it1 = S.begin(); it1 != S.end(); it1++)
  {
    std::cout << " " << *it1;
  }
  std::cout << std::endl;
}

int main()
{
  std::set<unsigned int> S;

  for(unsigned int i = 0; i < 10; i++)
    {
    S.insert(i);
    }

  Output(S);

  /*
  // manual removal
  for(std::set<unsigned int>::iterator it1 = S.begin(); it1 != S.end(); it1++)
  {
    if(*it1 == 4)
    {
      S.erase(it1);
    }
  }
  */
  std::set<unsigned int>::iterator it;
  it = S.find(4);
   if(it!=S.end())
    {
    S.erase(it);
    }

  Output(S);

  return 0;
}
