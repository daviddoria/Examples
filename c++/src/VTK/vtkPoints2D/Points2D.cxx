#include <vtkPoints2D.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPoints2D> points = vtkSmartPointer<vtkPoints2D>::New();
  points->InsertNextPoint(0,0);
  points->InsertNextPoint(1,1);
  
  double bounds[4];
  points->GetBounds(bounds);
  
  cout << "Bounds: " << bounds[0] << " " << bounds[1] << " " << bounds[2] << " " << bounds[3] << vtkstd::endl;
  
  return 0;
}
