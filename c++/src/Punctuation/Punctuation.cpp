#include <iostream>
#include <cctype>
#include <string>

int main()
{
  std::string teststring = "test.string";

  for(unsigned int i = 0; i < teststring.size(); i++)
    {
    if(!ispunct(teststring[i]))
      {
      std::cout << teststring[i];
      }
    }

  return 0;
}
