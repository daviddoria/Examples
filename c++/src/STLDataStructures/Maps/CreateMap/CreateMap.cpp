#include <iostream>
#include <map>
#include <string>

int main(int argc, char *argv[])
{
  // Create a map from strings to doubles
  std::map <std::string, double> myMap;
  
  // Create a mapping from "testone" to 111
  myMap.insert(std::pair<std::string, double>("testone", 111));
  
  // Key to value lookup: Method 1
  std::cout << myMap["testone"] << std::endl;
  
  // Key to value lookup: Method 2
  // Create an iterator
  std::map<std::string, double>::iterator iter = myMap.find("testone");
  
  if(iter == myMap.end())
  {
    std::cout << "Not found." << std::endl;
  }
  else
  {
      std::cout << "Found: " << iter->second << std::endl;	
  }
  
  return 0;
}
