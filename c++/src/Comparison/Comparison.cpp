#include <iostream>

int main(int argc, char *argv[])
{
{
  int i = 3;

  if(i < 4)
  {
    std::cout << "test" << std::endl;
  }
}

{
  unsigned int i = 3;
  if(i < 3u)
  {
    std::cout << "test" << std::endl;
  }
}

{
  unsigned long i = 3;
  if(i < 3u)
  {
    std::cout << "test" << std::endl;
  }
}


  return 0;
}
