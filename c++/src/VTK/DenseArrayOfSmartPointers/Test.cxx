#include <vtkSmartPointer.h>
#include <vtkDenseArray.h>

#include "vtkRay.h"

int main(int argc, char *argv[])
{
      
  vtkSmartPointer<vtkDenseArray<vtkSmartPointer<vtkRay> > > array = vtkSmartPointer<vtkDenseArray<vtkSmartPointer<vtkRay> > >::New();
  array->Resize(5,5);
  
  for(unsigned int i = 0; i < 5; i++)
  {
    for(unsigned int j = 0; j < 5; j++)
    {
      vtkSmartPointer<vtkRay> Ray = vtkSmartPointer<vtkRay>::New();
      double origin[3] = {0.0, 0.0, 0.0};
      double direction[3] = {1.0, 0.0, 0.0};
      Ray->SetOrigin(origin);
      Ray->SetDirection(direction);
  
      array->SetValue(i, j, Ray);
  
      vtkRay* RetrievedRay = array->GetValue(i,j);
      vtkstd::cout << *(RetrievedRay) << vtkstd::endl;
    }
  }
  
  for(unsigned int i = 0; i < 5; i++)
  {
    for(unsigned int j = 0; j < 5; j++)
    {
      vtkRay* RetrievedRay = array->GetValue(i,j);
      vtkstd::cout << *(RetrievedRay) << vtkstd::endl;
    }
  }
  
  return 0;
}
