#include "Point.h"

#include <iostream>
#include <vector>

template <>
void Output<double>(std::vector<double> &V)
{
  std::cout << std::endl << "double" << std::endl << "----------" << std::endl;
  for(unsigned int i = 0; i < V.size(); i++)
  {
    std::cout << V[i] << std::endl;
  }

}

template <>
void Output<unsigned int>(std::vector<unsigned int> &V)
{
  std::cout << std::endl << "unsigned int" << std::endl << "----------" << std::endl;
  for(unsigned int i = 0; i < V.size(); i++)
  {
    std::cout << V[i] << std::endl;
  }

}