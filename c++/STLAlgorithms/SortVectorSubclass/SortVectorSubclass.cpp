#include <iostream>
#include <vector>
#include <algorithm>

class MyClass : public std::vector<int>
{
  double otherStuff;
};

bool SpecialSort(const int& value1, const int& value2)
{
  return (value1 < value2);
}

int main(int, char *[])
{
  MyClass test;
  test.push_back(1);
  test.push_back(2);
  test.push_back(3);

  //std::sort(test.begin(), test.end());
  std::sort(test.begin(), test.end(), SpecialSort);
  return 0;
}
