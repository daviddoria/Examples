#include "itkMersenneTwisterRandomVariateGenerator.h"

void UniformDouble();
void Int();

int main(int, char*[])
{
  //UniformDouble();
  Int();

  return EXIT_SUCCESS;
}

void UniformDouble()
{
  typedef itk::Statistics::MersenneTwisterRandomVariateGenerator GeneratorType;
  GeneratorType::Pointer generator = GeneratorType::New();

  generator->Initialize();
  std::cout << generator->GetUniformVariate(0, 5) << std::endl;
}

void Int()
{
  typedef itk::Statistics::MersenneTwisterRandomVariateGenerator GeneratorType;
  GeneratorType::Pointer generator = GeneratorType::New();

  generator->Initialize();
  std::cout << generator->GetIntegerVariate(5) << std::endl; // Get an int between 0 and 5 (inclusive - that is sample from the set {0,1,2,3,4,5})
}