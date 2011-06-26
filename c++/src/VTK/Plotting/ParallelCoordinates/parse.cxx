#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>

std::vector<std::string> split(const std::string &s, char delim);

int main(int argc, char*argv[])
{

  std::ifstream fin(argv[1]);

  if(fin == NULL)
    {
    std::cout << "Cannot open file." << std::endl;
    }

  std::vector<std::string> lines;
  std::string line;

  while(getline(fin, line))
  {
    lines.push_back(line);
  }

std::vector<std::string> names;
std::vector<std::string> mpg;
std::vector<std::string> cyl;
std::vector<std::string> dsp;
std::vector<std::string> hp;
std::vector<std::string> lbs;
std::vector<std::string> acc;
std::vector<std::string> year;
std::vector<std::string> origin;
// there are 9 categories
  //for(unsigned int i = 0; i < lines.size(); i++)
  for(unsigned int i = 0; i < 1; i++)
    {
    std::string currentLine = lines[i];
    
    std::vector<std::string> words = split(currentLine, '{');
    currentLine = words[1];
    words.clear();
    words = split(currentLine, '}');
    currentLine = words[0];

    words = split(currentLine, ',');

  
    std::vector<std::vector<std::string> > things;
    for(unsigned int j = 0; j < words.size(); j++)
      {
      std::vector<std::string> thing = split(words[j], ':');
      things.push_back(thing);
      }

    if(things.size() != 9)
    {
      std::cerr << "Error: size is " << things.size() << std::endl;
      exit(-1);
    }
    names.push_back(things[0][1]);
    mpg.push_back(things[1][1]);
    cyl.push_back(things[2][1]);
    dsp.push_back(things[3][1]);
    hp.push_back(things[4][1]);
    lbs.push_back(things[5][1]);
    acc.push_back(things[6][1]);
    year.push_back(things[7][1]);
    origin.push_back(things[8][1]);
    }

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