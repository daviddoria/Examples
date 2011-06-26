#if VTK_MAJOR_VERSION>5 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>4)
#define a 1
#else
#define a 2
#endif

#include <vtkPolyData.h>

int main(int, char *[])
{
  std::cout << a << std::endl;

  return EXIT_SUCCESS;
}
