#include "itkSize.h"

void Method1();
void Method2();
void Method3();

int main(int, char *[])
{
  Method1();
  Method2();
  Method3();

  return EXIT_SUCCESS;
}

void Method1()
{
  itk::Size<2> size;
  size.Fill(0);
  std::cout << size << std::endl;
}

void Method2()
{
  itk::Size<2> size;
  size[0] = 1;
  size[1] = 2;
  std::cout << size << std::endl;
}

void Method3()
{
  // itk::Size<2> size(4); // this doesn't work
  // std::cout << size << std::endl;
}

