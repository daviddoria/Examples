#include <iostream>
#include <vector>

#include "Point.h"

int main(int argc, char* argv[])
{
  std::vector<unsigned int> A;
  for(unsigned int i = 0; i < 3; i++)
  {
    A.push_back(i * 1.3);
  }

  Output(A);
  Output<unsigned int>(A);

  std::vector<double> B;
  for(unsigned int i = 0; i < 3; i++)
  {
    B.push_back(i * 1.3);
  }

  Output(B);
  Output<double>(B);
  
  std::vector<int> C;
  for(unsigned int i = 0; i < 3; i++)
  {
    C.push_back(i * 1.3);
  }

  Output(C);
  Output<int>(C);

  return 0;
}
