#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cstdlib>

void ReadInt(const std::string &Filename);
void ReadIntStringDouble(const std::string &Filename);
void VectorOfLines(const std::string &Filename);
void SkipComments(const std::string &Filename);


/* Example.txt
23
test
4.5
*/

int main(int argc, char *argv[])
{
  std::string Filename = argv[1];

  ReadInt(Filename);
  //ReadIntStringDouble(Filename);
  //VectorOfLines(Filename);
  //SkipComments(Filename);
  return 0;
}

void ReadInt(const std::string &Filename)
{
  std::cout << "Filename: " << Filename << std::endl;

  int Integer;

  std::ifstream fin(Filename.c_str());
  
  if(!fin )
  {
  std::cout << "File not found!" << std::endl;
  exit(-1);
  }

  std::string line;
  std::stringstream linestream;
	  
  getline(fin, line);
  linestream.clear();
  linestream << line;
  linestream >> Integer;	

  fin.close();

  std::cout << "Integer: " << Integer << std::endl;
}

void ReadIntStringDouble(const std::string &Filename)
{
  std::cout << "Filename: " << Filename << std::endl;

  int Integer;
  std::string String;	
  double Double;

  std::ifstream fin(Filename.c_str());


  if(fin == NULL)
  {	
    std::cout << "Cannot open file." << std::endl;
  }

  std::string line;
  std::stringstream linestream;
	  
  getline(fin, line);
  linestream.clear();
  linestream << line;
  linestream >> Integer;	

  getline(fin, line);
  linestream.clear();
  linestream << line;
  linestream >> String;

  getline(fin, line);
  linestream.clear();
  linestream << line;
  linestream >> Double;

  fin.close();

  std::cout << "Integer: " << Integer << std::endl;
  std::cout << "String: " << String << std::endl;
  std::cout << "Double: " << Double << std::endl;
}

void VectorOfLines(const std::string &Filename)
{
	
  std::ifstream fin(Filename.c_str());

  if(fin == NULL)
  {
    std::cout << "Cannot open file." << std::endl;
  }

  std::vector<std::string> Lines;
  std::string line;
  
  while(getline(fin, line))
  {
    Lines.push_back(line);
  }
	
  for(unsigned int i = 0; i < Lines.size(); i++)
  {
    std::cout << Lines[i] << std::endl;
  }

}

void SkipComments(const std::string &Filename)
{
  /* test.txt
  P3
  # example comment
  512 512
  255

  */
  std::ifstream fin(Filename.c_str());

  if(fin == NULL)
  {
    std::cout << "Cannot open file." << std::endl;
  }

  char MV[2];//the first 2 ascii values
  int width, height;
  int MPV;//max pixel value

  std::string line;
	  
  getline(fin, line); //get first line
	  
  MV[0] = line[0];
  MV[1] = line[1];
  std::cout << "MV[0] = " << MV[0] << std::endl;
  std::cout << "MV[1] = " << MV[1] << std::endl;
  
  line = "";
  getline(fin, line); //get second line
  std::string FirstChar = line.substr(0, 1);

  if(FirstChar == "#")
  {
    std::cout << "Skipped this line." << std::endl;
  }
  
  line = "";
  getline(fin, line); //get third line
  std::stringstream LineStream;
  LineStream << line;
  LineStream >> width >> height;
  std::cout << "Width = " << width << std::endl;
  std::cout << "Height = " << height << std::endl;
  LineStream.str("");
  
  line = "";
  getline(fin, line); //get fourth line
	  
  LineStream.str("");
  LineStream.clear();
  LineStream << line;
  LineStream >> MPV;
	  
  std::cout << "MPV = " << MPV << std::endl;
  
}