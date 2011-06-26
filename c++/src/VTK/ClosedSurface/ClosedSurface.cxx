#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkFeatureEdges.h>

int main(int argc, char *argv[])
{
  
  //Create a sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();

  vtkSmartPointer<vtkFeatureEdges> featureEdges = 
      vtkSmartPointer<vtkFeatureEdges>::New();
  featureEdges->FeatureEdgesOff();
  featureEdges->BoundaryEdgesOn();
  featureEdges->NonManifoldEdgesOn();
  featureEdges->SetInputConnection(sphereSource->GetOutputPort());
  featureEdges->Update();
  
  int numberOfOpenEdges = featureEdges->GetOutput()->GetNumberOfCells();
  
  if(numberOfOpenEdges > 0)
    {
    cout << "Surface is not closed" << endl;
    }
  else
    {
    cout << "Surface is closed" << endl;
    }
    
  return EXIT_SUCCESS;
}