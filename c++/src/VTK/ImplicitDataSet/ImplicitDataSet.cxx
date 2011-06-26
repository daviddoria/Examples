#include <vtkSmartPointer.h>
#include <vtkImplicitDataSet.h>
#include <vtkSphereSource.h>

int main(int argc, char **argv)
{
  
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
      
  vtkSmartPointer<vtkImplicitDataSet> implicitSphere = 
      vtkSmartPointer<vtkImplicitDataSet>::New();
  implicitSphere->SetDataSet(sphereSource->GetOutput());
  
  double x[3] = {.5,0,0};
  cout << "x: " << implicitSphere->EvaluateFunction(x) << endl;
  
  return EXIT_SUCCESS;
}
