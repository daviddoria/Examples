#include <iostream>
#include <string>
#include <cstdlib>
#include <sstream>
#include <fstream>

int main(int argc, char *argv[])
{
  if(argc != 2)
  {
    std::cout << "Required parameters: InputFilename" << std::endl;
    exit(-1);
  }

  std::string InputFilename = argv[1];

  std::ifstream in(InputFilename.c_str());

  if (!in)
  {
    std::cout << "File " << InputFilename << " not found" << std::endl;
    exit(-1);
  }

#if 0 //bad way
  std::string line;
  while(getline(in, line))
  {
    if(line[0] == '#') //we found the first line we were looking for, need to switch to the next loop
    {
      break;
    }
  }

  //start reading data - //woops - we missed the first line of interests (here the line starting with #)
  while(getline(in, line))
  {
    std::cout << line << std::endl;
  }
#else
  std::string line;

  while(getline(in, line))
  {
    if(line[0] == '#') //we found the first line we were looking for, need to switch to the next loop
    {
      break;
    }
  }

  //start reading data - //woops - we missed the first line of interests (here the line starting with #)
  do
  {
    std::cout << line << std::endl;
  }while(getline(in, line));
#endif

  return 0;
}
