#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <iterator>
#include <vector>

int main(int argc, char *argv[])
{
  std::string sentence = "hello world test 1 2 3";
  
  std::istringstream iss(sentence);
  std::vector<std::string> words;
  std::copy(std::istream_iterator<std::string>(iss),
             std::istream_iterator<std::string>(),
             std::back_inserter<std::vector<std::string> >(words));
  
  for(unsigned int i = 0; i < words.size(); i++)
  {
    std::cout << words[i] << std::endl;
  }
  
  
  return 0;
}
