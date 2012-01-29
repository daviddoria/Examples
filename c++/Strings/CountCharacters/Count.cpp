#include <iostream>
#include <algorithm>
#include <string>

int main()
{
  std::string content = "A table and A chair and A desk";
  char letterToFind = 'A'; 
  int count = std::count(content.begin(),content.end(),letterToFind); 
  std::cout << count << " occurrence of " << letterToFind << std::endl;

  return 0;
}
