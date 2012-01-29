#include <iostream>
#include <vector>
#include <cstdlib>

#include <cmath>
#include <boost/progress.hpp>

void LongFunction();
void DoubleLoop();
void Conditional(const bool show);	

int main(int argc, char* argv[])
{
  //LongFunction();
  //DoubleLoop();
  Conditional(false);
  Conditional(true);
  return 0;
}

void LongFunction()
{
  unsigned int BigNum = 1e9;
  boost::progress_display show_progress(BigNum);
  
  double temp;
  for(unsigned int i = 0; i < BigNum; i++)
  {
	  temp = sin(i) / i;
	  ++show_progress;
  }
}

void DoubleLoop()
{
  int X = 1000;
  int Y = 1000;
  boost::progress_display show_progress(X*Y);
  
  double temp;
  for(int x = 0; x < X; x++)
  {
    for(int y = 0; y < Y; y++)
    {
      temp = sin(y) / y;
      ++show_progress;
    }
  }
}

void Conditional(const bool show)
{
  unsigned int NumIters = 1e7;

  boost::progress_display * show_progress = NULL;
  if(show)
  {
    show_progress = new boost::progress_display(NumIters);
  }

  double temp;
  for(unsigned int i = 0; i < NumIters; i++)
  {
    temp = sin(i) / i;
    if(show)
    {
      ++(*show_progress);
    }
  }

  std::cout << "done." << std::endl;
}
