#include <vtkSmartPointer.h>
#include <vtkCoincidentPoints.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkCoincidentPoints> coincidentPoints = 
    vtkSmartPointer<vtkCoincidentPoints>::New();
  
  return EXIT_SUCCESS;
}