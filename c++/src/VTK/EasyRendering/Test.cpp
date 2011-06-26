#include "vtkPolyData.h"
#include "vtkSphereSource.h"
#include "vtkSmartPointer.h"

#include "EasyRenderer.h"

int main ()
{

  vtkSmartPointer<vtkSphereSource> SphereSource1 = vtkSmartPointer<vtkSphereSource>::New();
  SphereSource1->SetCenter(0.0, 0.0, 0.0);
  SphereSource1->SetRadius(1.0);
  vtkPolyData* Sphere1 = SphereSource1->GetOutput();
  
  vtkSmartPointer<vtkSphereSource> SphereSource2 = vtkSmartPointer<vtkSphereSource>::New();
  SphereSource2->SetCenter(5.0, 0.0, 0.0);
  SphereSource2->SetRadius(2.0);
  vtkPolyData* Sphere2 = SphereSource2->GetOutput();
  
  EasyRenderer MyEasyRenderer;
  MyEasyRenderer.AddObject(Sphere1);
  MyEasyRenderer.AddObject(Sphere2);
  MyEasyRenderer.Render();
  
  return 0;
}