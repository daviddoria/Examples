#include <iostream>
#include <iterator>
#include <list>

int main () 
{
  std::list<int> firstlist, secondlist;
  for (int i=1; i<=5; i++)
  { 
    firstlist.push_back(i); 
    secondlist.push_back(i*10); 
  }

  std::list<int>::iterator it;
  it = firstlist.begin();
  advance (it,3);

  copy (secondlist.begin(),secondlist.end(),inserter(firstlist,it));

  for ( it = firstlist.begin(); it!= firstlist.end(); ++it )
  {
    std::cout << *it << " ";
  }
  std::cout << std::endl;

  return 0;
}