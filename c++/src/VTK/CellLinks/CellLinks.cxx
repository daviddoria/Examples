#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkProperty.h>
#include <vtkCellLinks.h>

int main(int, char *[])
{
   //create an image data
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  vtkSmartPointer<vtkCellLinks> cellLinksFilter = 
      vtkSmartPointer<vtkCellLinks>::New();
  cellLinksFilter->BuildLinks(sphereSource->GetOutput());
  
  /*
  vtkCellLinks::Link link = cellLinksFilter->GetLink(0);
  
  cout << "There are " << link.ncells << endl;
  
  cout << "Point 0 is used by cells ";
  
  for(unsigned int i = 0; i < link.ncells; i++)
    {
    cout << link.cells[i] << endl;
    }
  */  
  
  vtkIdType* cells = cellLinksFilter->GetCells(0);
  
  cout << "Point 0 is used by cells ";
  
  for(unsigned int i = 0; i < 5; i++)
    {
    cout << cells[i] << endl;
    }
  
  return EXIT_SUCCESS;
}
