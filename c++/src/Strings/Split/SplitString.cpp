#include <iostream>
#include <sstream>
#include <string>
#include <algorithm>
#include <iterator>
#include <vector>
 
void SplitOnSpaces();
void SplitOnDelim();
std::vector<std::string> split(const std::string &s, char delim) ;
 
int main(int argc, char *argv[])
{
  SplitOnSpaces();
  //SplitOnDelim();
 
  return 0;
}
 
std::vector<std::string> split(const std::string &s, char delim) 
{
  std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while(std::getline(ss, item, delim))
    {
        elems.push_back(item);
    }
    return elems;
}
 
 
void SplitOnDelim()
{
  std::cout << "SplitOnDelim" << std::endl << "--------" << std::endl;
  std::string sentence = "hello,this,is,a,test";
 
  std::vector<std::string> words = split(sentence, ',');
 
  for(unsigned int i = 0; i < words.size(); i++)
  {
    std::cout << words[i] << std::endl;
  }
}
 
void SplitOnSpaces()
{
  std::cout << "SplitOnSpaces" << std::endl << "--------" << std::endl;
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
}
