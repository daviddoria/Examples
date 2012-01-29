#include <iostream>
#include <vector>

#include <algorithm>

int main(int, char*[])
{
  std::vector<unsigned int> numbers;
  numbers.push_back(1);
  numbers.push_back(2);
  numbers.push_back(3);
  numbers.push_back(4);

  // Reverse the elements in the vector
  std::reverse (numbers.begin( ), numbers.end( ) );

  for(unsigned int i = 0; i < numbers.size(); ++i)
  {
    std::cout << numbers[i] << std::endl;
  }
  
  return 0;
}
