#include <vtkSmartPointer.h>
#include <vtkFloatArray.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataNormals.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
    
  // generate normals
  vtkSmartPointer<vtkPolyDataNormals> normalGenerator = 
      vtkSmartPointer<vtkPolyDataNormals>::New();
  normalGenerator->SetInputConnection(sphereSource->GetOutputPort());
  normalGenerator->Update();
  
  {
  vtkSmartPointer<vtkFloatArray> normals = 
      vtkFloatArray::SafeDownCast(normalGenerator->GetOutput()->GetPointData()->GetArray("Normals"));
  if(normals)
    {
    cout << "Got normals by name." << endl;
    }
  }
  
  {
  vtkSmartPointer<vtkFloatArray> normals = 
      vtkFloatArray::SafeDownCast(normalGenerator->GetOutput()->GetPointData()->GetNormals());
  if(normals)
    {
    cout << "Got normals by special array." << endl;
    }
  }
  
  return EXIT_SUCCESS;
}
