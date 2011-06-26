#include <vtkVersion.h>
#include <sstream>

int main(int, char *[])
{

  std::cout << vtkVersion::GetVTKSourceVersion() << std::endl;

  std::cout << vtkVersion::GetVTKMajorVersion() << std::endl;
  std::cout << vtkVersion::GetVTKMinorVersion() << std::endl;

  return EXIT_SUCCESS;
}
