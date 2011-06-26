#include <vtkSmartPointer.h>
#include <vtkRungeKutta4.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkRungeKutta4> RungeKutta4 = 
    vtSkSmartPointer<vtkRungeKutta4>::New();
  
  return EXIT_SUCCESS;
}