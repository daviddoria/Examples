#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <cstdlib>
#include <stdexcept>

// You either must manually iterate over the map, or usse a boost::bimap
std::string FindKeyByValue(std::map <std::string, int> MyMap, int value);

int main(int argc, char *argv[])
{
  // Create a map
  std::map <std::string, int> MyMap;

  // Create a mapping from "test" to 111
  MyMap["test"] = 111;

  // Create an iterator
  std::map<std::string,int>::iterator iter;

  // Try to find "test"
  iter = MyMap.find("test");
  if(iter != MyMap.end())
    {
    // output the value that "testone" maps to
    std::cout << "Found!" << std::endl;
    std::cout << iter->first << " " << iter->second << std::endl;
    }
  else
    {
    std::cout << "Not found!" << std::endl;
    }

  std::cout << MyMap["test"] << std::endl;

  std::cout << "FindKeyByValue: " << FindKeyByValue(MyMap, 111) << std::endl;
  return 0;
}

std::string FindKeyByValue(std::map <std::string, int> MyMap, int value)
{
  std::map<std::string, int>::const_iterator it;
  for (it = MyMap.begin(); it != MyMap.end(); ++it)
    {
    if (it->second == value)
      {
      return it->first;
      break;
      }
    }
  throw std::runtime_error("Not found!");
  
  // Prevent compiler warning about no return value. This should never be reached.
  return std::string();
}
