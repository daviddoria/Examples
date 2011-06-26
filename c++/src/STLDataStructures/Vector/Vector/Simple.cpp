#include <iostream>
#include <vector>
#include <algorithm>

int main(int argc, char* argv[])
{
  std::vector<unsigned int> V(10);
  
  for(unsigned int i = 0; i < 10; i++)
  {
    V[i] = 10*i;
    std::cout << V[i]   << " ";
  }
  
  std::cout << std::endl;
  
  std::vector<unsigned int>::iterator it = find(V.begin(), V.end(), 70);
  
  if(it == V.end())
  {
      std::cout << "Could not find 70 in the vector"  << std::endl;
  }
  else
  {
    std::cout << "The number 70 is located at index " << it - V.begin() << std::endl;
  }
  
  return 0;
}
