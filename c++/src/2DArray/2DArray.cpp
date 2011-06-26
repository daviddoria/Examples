#include <iostream>

using namespace std;

void FixedLength();
void VariableLength();

void Fill(void* buffer, unsigned int Width, unsigned int Height);
void Read(void* buffer, unsigned int Width, unsigned int Height);

int main()
{
  FixedLength();
  VariableLength();
  
  return 0;
}

void Fill(void* buffer, unsigned int Width, unsigned int Height)
{
  char* buff = static_cast<char*> (buffer);
  for(unsigned int i = 0; i < Width; ++i)
  {
    for(unsigned int j = 0; j < Height; ++j)
    {
	    buff[Width*i + Height*j + 0] = 0;
	    buff[Width*i + Height*j + 1] = 1;
	    buff[Width*i + Height*j + 2] = 2;
    }
  }
}

void Read(void* buffer, unsigned int Width, unsigned int Height)
{
  unsigned char r,g,b;
  
  char * buff = static_cast<char*> (buffer);
  for(unsigned int i = 0; i < Width; ++i)
  {
    for(unsigned int j = 0; j < Height; ++j)
    {
      r = buff[i*Width + j*Height + 0];
      g = buff[i*Width + j*Height + 1];
      b = buff[i*Width + j*Height + 2];
      cout << static_cast<int>(r) << " " << static_cast<int>(g) << " " << static_cast<int>(b) << endl;
    }
  }
  cout << endl; 
}

void FixedLength()
{
  cout << "Fixed Length:" << endl << "-------------" << endl;
  unsigned int Width = 4;
  unsigned int Height = 4;
  
  unsigned char buffer[4][4][3];
  
  Fill(buffer, Width, Height);
  Read(buffer, Width, Height);

}

void VariableLength()
{
  cout << "Variable Length:" << endl << "-------------" << endl;
  unsigned int Width = 4;
  unsigned int Height = 4;
  unsigned int d1 = Width, d2 = Height;
  //Want to do this: unsigned char bufImage[Width][Height][3];

  unsigned char* buffer = new unsigned char[Width*Height*3];
  
  for(unsigned int i = 0; i < Width; ++i)
  {
    for(unsigned int j = 0; j < Height; ++j)
    {
      buffer[i*d1+j*d2+0] = 0;
      buffer[i*d1+j*d2+1] = 0;
      buffer[i*d1+j*d2+2] = 0;
    }
  }
  
  Fill(buffer, Width, Height);
  Read(buffer, Width, Height);
}