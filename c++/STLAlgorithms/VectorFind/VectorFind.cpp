#include <iostream>
#include <vector>
#include <algorithm>

int main()
{
  typedef std::vector<int> VectorType;
  VectorType v;
  v.push_back(10);
  v.push_back(20);
  v.push_back(30);

  const int value = 20;
  VectorType::iterator it = find(v.begin(), v.end(), value);

  if(it == v.end())
  {
    std::cout << "Could not find " << value << " in the vector"  << std::endl;
  }
  else
  {
    std::cout << "The number " << value << " is located at index " << it - v.begin() << std::endl;
  }
  return 0;
}
