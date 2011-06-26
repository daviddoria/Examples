#include <vtkSmartPointer.h>
#include <vtkFixedPointRayCastImage.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkFixedPointRayCastImage> image = 
    vtkSmartPointer<vtkFixedPointRayCastImage>::New();
  
  return EXIT_SUCCESS;
}
