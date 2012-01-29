#include <iostream>
#include <vector>

int main(int, char*[])
{
  std::vector<int> v(10);

  for(unsigned int i = 0; i < 10; i++)
    {
    v[i] = i;
    }
  
  try
  {
    // Statements that may throw exceptions you want to handle now go here
    //std::cout << v[-1] << std::endl;
    std::cout << v.at(-1) << std::endl;
  }
  catch (...)
  {
    std::cerr << "We caught an exception of unknown type." << std::endl;
  }

  return 0;
}
