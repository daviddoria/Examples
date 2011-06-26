#include <vtkSmartPointer.h>
#include <vtkDenseArray.h>

#include "vtkRay.h"

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkRay> Ray = vtkSmartPointer<vtkRay>::New();
  double origin[3] = {0.0, 0.0, 0.0};
  double direction[3] = {1.0, 0.0, 0.0};
  Ray->SetOrigin(origin);
  Ray->SetDirection(direction);
      
  vtkSmartPointer<vtkDenseArray<vtkRay*> > array = vtkSmartPointer<vtkDenseArray<vtkRay*> >::New();
  array->Resize(5,5);
  
  array->SetValue(4,4, Ray);
  
  vtkRay* RetrievedRay = array->GetValue(4,4);
  //vtkstd::cout << RetrievedPoint.x << " " << RetrievedPoint.y << vtkstd::endl;
  return 0;
}
