#include <itkImageRandomNonRepeatingConstIteratorWithIndex.h>

int main(int, char*[])
{
  itk::RandomPermutation rp(5);
  rp.Dump();
  std::cout << std::endl;

  for(unsigned int i = 0; i < 5; i++)
    {
    std::cout << rp[i] << " ";
    }
  std::cout << std::endl << std::endl;

  rp.Shuffle();
  rp.Dump();


  return EXIT_SUCCESS;
}
