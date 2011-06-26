#include <iostream>
#include <vector>
#include <algorithm>
#include <string>

void Output(std::vector<std::string> &Strings);
void Alphabetize(std::vector<std::string> &Strings);
    
int main (int argc, char *argv[]) 
{
    std::vector<std::string> Strings;
    Strings.push_back("Hayley");
    Strings.push_back("David");
    Strings.push_back("Tony");
    Output(Strings);
    
    Alphabetize(Strings);
    Output(Strings);

    return 0;
}

void Alphabetize(std::vector<std::string> &Strings)
{
  std::sort(Strings.begin(), Strings.end());
}

void Output(std::vector<std::string> &Strings)
{
  for(unsigned int i = 0; i < Strings.size(); i++)
  {
    std::cout << Strings[i] << std::endl;
  }
}
