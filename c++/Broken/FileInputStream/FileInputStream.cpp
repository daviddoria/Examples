#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cstdlib> //for exit()

int main(int argc, char *argv[])
{
  std::string Filename = argv[1];

  std::ifstream fin(Filename.c_str());
  if(!fin )
  {
    std::cout << "File not found!" << std::endl;
    exit(-1);
  }

  //why does it print the last value twice?
  while(!fin.eof())
  {    
    double a;
    fin >> a;
    std::cout << a << std::endl;
  }
  
  fin.close();

  return 0;
}
