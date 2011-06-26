#include <vtkMath.h>

int main(int argc, char *argv[])
{
  //The number of random numbers we wish to produce
  unsigned int NumRand = 3;
  
  //without this line, the random numbers will be the same every iteration
  vtkMath::RandomSeed(time(NULL));
  
  //Generate NumRand random numbers from a uniform distribution between 0.0 and 2.0
  for(unsigned int i = 0; i < NumRand; i++)
    {
    double a = vtkMath::Random(0.0,2.0);
    std::cout << a << std::endl;
    }
  
  return 0;
}
