#include <vtkSphereSource.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

int main()
{
  //setup points (geometry)
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  
  vtkPolyData* sphere = sphereSource->GetOutput();
  
  double bounds[6];
  sphere->GetBounds(bounds);
  
  cout << "xmin: " << bounds[1] << " " 
       << "xmax: " << bounds[1] << " " 
       << "ymin: " << bounds[2] << " " 
       << "ymax: " << bounds[3] << " " 
       << "zmin: " << bounds[4] << " " 
       << "zmax: " << bounds[5] << endl;
  
  return EXIT_SUCCESS;
}
