#include <vtkSmartPointer.h>
#include <vtkPoints.h>

int main(int argc, char *argv[])
{
  //create a list of points
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  
  //add a point to the list
  const float P[3] = {1.0, 2.0, 3.0};
  Points->InsertNextPoint(P);
  
  //retrieve the point
  double* Pout = Points->GetPoint(0);
  vtkstd::cout << Pout[0] << " " << Pout[1] << " " << Pout[2] << vtkstd::endl;
  
  return 0;
}
