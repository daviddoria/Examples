#include <iostream>
#include <vector>
#include <regex>

int main()
{
  //std::regex rx("ello");
  std::regex rx("*ello");
  std::string str = "Hello world";

  bool hasMatch = regex_match(str.begin(), str.end(), rx);
  std::cout << "hasMatch? " << hasMatch << std::endl;

  bool hasSearchMatch = regex_search(str.begin(), str.end(), rx);
  std::cout << "hasSearchMatch? " << hasSearchMatch << std::endl;
  return 0;
}
