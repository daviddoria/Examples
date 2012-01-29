#include <iostream>
#include <set>
#include <algorithm>

int main()
{
  std::set<int> a;
  a.insert(0);
  a.insert(1);
  a.insert(2);
  std::set<int> b;
  b.insert(2);
  b.insert(3);
  b.insert(4);
  std::set<int> result;

  std::set_union(a.begin(), a.end(), b.begin(), b.end(),
            std::inserter<std::set<int> >(result, result.begin()));
  
  std::cout << "a.size: " << a.size() << std::endl;
  std::cout << "b.size: " << b.size() << std::endl;
  std::cout << "result.size: " << result.size() << std::endl;

  return 0;
}