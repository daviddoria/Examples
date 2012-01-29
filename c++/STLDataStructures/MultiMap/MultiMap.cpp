#include <iostream>
#include <map>

int main(int, char *[])
{
	
  std::multimap <int, double> MyMap;
  
  //create a mapping from 1 to 1.2
  MyMap.insert(std::pair<int, double>(1, 1.2));
  
  //create an iterator
  std::map<int, double>::iterator iter;

  iter = MyMap.find(1);
      
  if(iter == MyMap.end())
  {
    std::cout << "Not found." << std::endl;
  }
  else
  {
    std::cout << "Found a map from " << iter->first << " to " << iter->second << std::endl;	
  }
  
  return 0;
}
