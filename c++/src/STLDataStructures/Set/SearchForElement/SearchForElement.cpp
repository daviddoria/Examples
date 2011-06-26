#include <iostream>
#include <set>
#include <algorithm>

int main()
{
  std::set<unsigned int> S;

  for(unsigned int i = 0; i < 10; i++)
    {
    S.insert(i);
    }

  //find the element '5' - it should be there!
  std::set<unsigned int>::iterator it;
  it = S.find(5);
  std::cout << "5 was " ;
  if(it!=S.end())
    {
    std::cout << " found!" << std::endl;
    }
  else
    {
    std::cout << " NOT found!" << std::endl;
    }

  //find the element '20' - it should NOT be there!
  it = S.find(20);
  std::cout << "20 was ";
  if(it!=S.end())
    {
    std::cout << " found!" << std::endl;
    }
  else
    {
    std::cout << " NOT found!" << std::endl;
    }

  return 0;
}
