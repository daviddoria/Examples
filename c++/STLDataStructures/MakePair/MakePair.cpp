#include <iostream>

int main (int argc, char *argv[])
{
  std::pair<std::string, int> a = std::make_pair("test", 7);

  std::cout << a.first << " " << a.second << std::endl;

  return 0;
}

