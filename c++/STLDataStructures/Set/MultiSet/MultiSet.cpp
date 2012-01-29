#include <iostream>
#include <set>

int main(int, char*[])
{
  std::multiset<int> S;

  // These elements get sorted upon insert, but duplicates are allowed.
  S.insert(2);
  S.insert(1);
  S.insert(1);

  for(std::multiset<int>::iterator it1 = S.begin(); it1 != S.end(); it1++)
    {
    std::cout << " " << *it1;
    }

  std::cout << std::endl;


  //erase the second element
  std::set<double>::iterator it1 = S.begin();
  it1++;
  S.erase(it1);

  Report(S);
  Output(S);

  S.clear();

  std::cout << "Size: " << S.size() << " Empty? " << S.empty() << std::endl;

  return 0;
}
