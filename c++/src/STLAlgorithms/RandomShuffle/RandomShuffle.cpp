#include <iostream>
#include <algorithm>
#include <vector>

int main(int argc, char* argv[])
{
  std::vector<int> v;
  
  //add 10 integers (0 to 9)
  for(int i = 0; i < 10; i++)
  {
    v.push_back(i);
  }
  
  std::cout << "Ordered" << std::endl;
  for(int i = 0; i < 10; i++)
  {
    std::cout << v[i] << " ";
  }
  
  std::random_shuffle(v.begin(), v.end());
  
  std::cout << std::endl << "Shuffled" << std::endl;
  for(int i = 0; i < 10; i++)
  {
    std::cout << v[i] << " ";
  }
	
  return 0;
}
