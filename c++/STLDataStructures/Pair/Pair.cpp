#include <iostream>
#include <vector>
#include <algorithm>

int main (int argc, char *argv[])
{
  std::pair<double, int> a(3.4, 7);

  std::cout << a.first << " " << a.second << std::endl;

  std::pair <int,int> two = std::make_pair (10,20);
  
  return 0;
}

