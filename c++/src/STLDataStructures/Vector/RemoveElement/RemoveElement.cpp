#include <iostream>
#include <vector>

void Output(std::vector<int>& v)
{
  for(unsigned int i = 0; i < v.size(); i++)
    {
    std::cout << v[i] << " ";
    }
  std::cout << std::endl;
}

int main(int, char*[])
{
  std::vector<int> v(10);

  for(unsigned int i = 0; i < 10; i++)
  {
    v[i] = i;
  }

  Output(v);

  for(std::vector<int>::iterator it = v.begin(); it!= v.end(); it++)
  {
    if(*it == 4)
      {
      v.erase(it);
      }
  }

  Output(v);
  
  return 0;
}
