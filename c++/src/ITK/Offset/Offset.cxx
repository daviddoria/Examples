#include "itkIndex.h"
#include "itkOffset.h"

#include <iostream>

int main(int argc, char *argv[])
{
  itk::Index<2> index;
  index[0] = 5;
  index[1] = 5;
  
  itk::Offset<2> offset;
  offset[0] = 1;
  offset[1] = 1;

  std::cout << offset << std::endl;

  std::cout << index + offset << std::endl;
  
  return EXIT_SUCCESS;
}