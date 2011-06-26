#include <vtkSmartPointer.h>
#include <vtkAssignAttribute.h>
#include <vtkSphereSource.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
    vtkSmartPointer<vtkSphereSource>::New();
    
  vtkSmartPointer<vtkAssignAttribute> vectors =
    vtkSmartPointer<vtkAssignAttribute>::New();
  vectors->SetInputConnection(sphereSource->GetOutputPort());
  vectors->Assign("Gradients", vtkDataSetAttributes::VECTORS,
                  vtkAssignAttribute::POINT_DATA);
 
  return EXIT_SUCCESS;
}