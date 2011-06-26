#include <iostream>
#include <fstream>

void Overwrite();
void Append();

int main(int argc, char *argv[])
{
  Overwrite();
  Append();
  return 0;
}

void Overwrite()
{
  //overwrite test.txt
  std::string Filename = "test.txt";
  std::ofstream fout(Filename.c_str());
  
  fout << 1 << std::endl << 2 << std::endl;

  fout.close();
  /*
  or
  ofstream fout;
  fout.open(Filename);
  */
}

void Append()
{
  std::string Filename = "append.txt";
  std::ofstream fout(Filename.c_str(), std::ios::app);
  
  fout << 1 << std::endl << 2 << std::endl;

  fout.close();
}