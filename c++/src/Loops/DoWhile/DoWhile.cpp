#include <iostream>
#include <vector>

int main(int argc, char *argv[])
{
  std::vector<int> vec;
  vec.push_back(0);
  vec.push_back(1);
  vec.push_back(2);
  vec.push_back(3);

  unsigned int i = 0;
  
  do
  {
    std::cout << vec[i] << std::endl;
    i++;
  }while(vec[i] < 1);

  return 0;
}
