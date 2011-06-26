#include "itkIndex.h"

int main(int, char *[])
{
  itk::Index<2> index;

  // Method 1
  index.Fill(0);

  /*
  // Method 2
  index[0] = 1;
  index[1] = 2;
  */

  std::cout << index << std::endl;

  return EXIT_SUCCESS;
}
