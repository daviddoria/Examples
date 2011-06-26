#include <vtkPoints.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0,0,0);
  
  double p[3];
  points->GetPoint(0,p);
  vtkstd::cout << "p: " << p[0] << " " << p[1] << " " << p[2] << vtkstd::endl;
  
  double newpoint[3] = {1.0, 1.0, 1.0};
  points->SetPoint(0, newpoint);
  
  double pnew[3];
  points->GetPoint(0,pnew);
  vtkstd::cout << "pnew: " << pnew[0] << " " << pnew[1] << " " << pnew[2] << vtkstd::endl;
  
  return 0;
}
