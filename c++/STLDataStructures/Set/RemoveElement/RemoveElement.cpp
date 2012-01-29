#include <iostream>
#include <set>
#include <algorithm>

void Output(std::set<unsigned int> S);

int main()
{
  std::set<unsigned int> S;

  for(unsigned int i = 0; i < 10; i++)
    {
    S.insert(i);
    }

  Output(S);

  int counter = 0;
  // you want to increment it1 before erasing it and remember the old value and then don't do anything in the increment part of the loop
//   for(std::set<unsigned int>::iterator it1 = S.begin(); it1 != S.end(); )
//   {
//     counter++;
//     std::set<unsigned int>::iterator it2 = it1++;
//     if(counter > 4)
//     {
//       S.erase(it2);
//     }
//   }

  for(std::set<unsigned int>::iterator it1 = S.begin(); it1 != S.end(); ++it1)
  {
    counter++;

    if(counter > 4)
    {
      //it1 = S.erase(it1);
      //it1 = erase(it1);
    }
  }

  Output(S);

  return 0;
}

void Output(std::set<unsigned int> S)
{
  // Output all of the elements in the set
  for(std::set<unsigned int>::iterator it1 = S.begin(); it1 != S.end(); it1++)
  {
    std::cout << " " << *it1;
  }
  std::cout << std::endl;
}