#include <vtkSmartPointer.h>
#include <vtkTransposeMatrix.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkTransposeMatrix> transposeMatrix = 
    vtkSmartPointer<vtkTransposeMatrix>::New();
    
  return EXIT_SUCCESS;
}