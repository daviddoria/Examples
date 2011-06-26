#include <vtkSmartPointer.h>
#include <vtkKdTreeSelector.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkKdTreeSelector> selector = 
    vtkSmartPointer<vtkKdTreeSelector>::New();
  return EXIT_SUCCESS;
}