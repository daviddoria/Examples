#include <vtkFloatingPointExceptions.h>

int main(int, char *[])
{
  vtkFloatingPointExceptions::Enable(); // disabled by default with gcc or visual studio, enabled by default by Borland CC.

  double x = 0.0;
  double y = 1.0/x; // floating-point exception

  std::cout << "x: " << x << " y: " << y << std::endl;

  return EXIT_SUCCESS;
}