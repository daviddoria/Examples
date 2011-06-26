#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkStripper.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  cout << "Number of cells before stripping: " << sphereSource->GetOutput()->GetNumberOfCells() << endl;
  
  vtkSmartPointer<vtkStripper> stripper = 
      vtkSmartPointer<vtkStripper>::New();
  stripper->SetInputConnection(sphereSource->GetOutputPort());
  stripper->Update();
  
  cout << "Number of cells after stripping: " << stripper->GetOutput()->GetNumberOfCells() << endl;
  
  return EXIT_SUCCESS;
}
