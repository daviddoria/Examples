#include "itkFixedArray.h"

int main(int, char *[])
{
  itk::FixedArray<double, 2> array;
  array[0] = 0;
  array[1] = 1;

  std::cout << array << std::endl;

  return EXIT_SUCCESS;
}
