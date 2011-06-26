#include <iostream>
#include <vector>

union MyUnion
{
  struct
  {
    int r,g,b;
  };
  int channels[3];
};

int main(int, char *[])
{
  MyUnion A;
  A.r = 2;

  std::cout << A.channels[0] << std::endl;

  return 0;
}
