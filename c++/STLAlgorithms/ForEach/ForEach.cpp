#include <iostream>
#include <algorithm>
#include <vector>

struct printFunctor
{
  void operator() (int x)
  {
    std::cout << x;
  }

};

int main()
{
  std::vector<int> v;
  v.push_back(1);
  v.push_back(2);
  v.push_back(3);

  printFunctor myPrint;
  std::for_each(v.begin(), v.end(), myPrint);

  return 0;
}