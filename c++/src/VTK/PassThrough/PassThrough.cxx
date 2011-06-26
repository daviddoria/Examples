#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkPassThrough.h>

int main(int argc, char *argv[])
{
  //Create a sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  cout << "Points before: " << sphereSource->GetOutput()->GetNumberOfPoints() << endl;
  
  vtkSmartPointer<vtkPassThrough> passThrough = 
      vtkSmartPointer<vtkPassThrough>::New();
  passThrough->SetInputConnection(sphereSource->GetOutputPort());
  passThrough->Update();
  
  vtkSmartPointer<vtkPolyData> output = vtkPolyData::SafeDownCast(passThrough->GetOutput());
  
  cout << "Points after: " << output->GetNumberOfPoints() << endl;
  
  return EXIT_SUCCESS;
}
