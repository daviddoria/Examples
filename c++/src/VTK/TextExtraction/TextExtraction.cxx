#include <vtkSmartPointer.h>
#include <vtkTextExtraction.h>

int main(int, char*[])
{
  vtkSmartPointer<vtkTextExtraction> textExtraction = 
    vtkSmartPointer<vtkTextExtraction>::New();
  
  return EXIT_SUCCESS;
}
