#include <vtkSmartPointer.h>
#include <vtkDenseArray.h>

#include "vtkRay.h"

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkDenseArray<vtkSmartPointer<vtkRay> > > array = vtkSmartPointer<vtkDenseArray<vtkSmartPointer<vtkRay > > >::New();
  array->Resize(5,1);
  
  for(unsigned int i = 0; i < 5; i++)
  {
    vtkSmartPointer<vtkRay> Ray = vtkSmartPointer<vtkRay>::New();
    vtkstd::cout << "Reference count after creation: " << Ray->GetReferenceCount() << vtkstd::endl;
    
    double origin[3] = {0.0, 0.0, 0.0};
    double direction[3] = {1.0, 0.0, 0.0};
    Ray->SetOrigin(origin);
    Ray->SetDirection(direction);

    array->SetValue(i, 0, Ray);
    vtkstd::cout << "Reference count after added to dense array: " << array->GetValue(i,0)->GetReferenceCount() << vtkstd::endl;
    vtkstd::cout << "Reference count after added to dense array: " << Ray->GetReferenceCount() << vtkstd::endl;
    
    vtkRay* RetrievedRay = array->GetValue(i,0);
    //vtkstd::cout << *(RetrievedRay) << vtkstd::endl;
  }
  
  for(unsigned int i = 0; i < 5; i++)
  {
    vtkRay* RetrievedRay = array->GetValue(i,0);
    vtkstd::cout << *(RetrievedRay) << vtkstd::endl;//segfault occurs here
  }
  
  return 0;
}
