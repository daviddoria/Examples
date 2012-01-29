#include <iostream>
#include <set>
#include <algorithm>

int main()
{
  std::set<unsigned int> s1;

  for(unsigned int i = 0; i < 10; i++)
    {
    s1.insert(i);
    }

  std::cout << "s1 size: " << s1.size() << std::endl;

  std::set<unsigned int> s2 = s1;

  for(unsigned int i = 11; i < 14; i++)
    {
    s2.insert(i);
    }

  std::cout << "s2 size: " << s2.size() << std::endl;

  std::set<unsigned int> s1butNots2;
  std::set_difference(s1.begin(), s1.end(), s2.begin(), s2.end(), inserter(s1butNots2, s1butNots2.begin()));

  std::cout << "s1butNots2 size: " << s1butNots2.size() << std::endl;

  std::set<unsigned int> s2butNots1;
  std::set_difference(s2.begin(), s2.end(), s1.begin(), s1.end(), inserter(s2butNots1, s2butNots1.begin()));

  std::cout << "s2butNots1 size: " << s2butNots1.size() << std::endl;

  return 0;
}
