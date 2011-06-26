#include <vtkSmartPointer.h>
#include <vtkMatrix3x3.h>

#include "vtkTestClass.h"

int main (int argc, char *argv[])
{
  
  vtkSmartPointer<vtkMatrix3x3> M = vtkSmartPointer<vtkMatrix3x3>::New();
  
  vtkSmartPointer<vtkTestClass> MyClass = vtkSmartPointer<vtkTestClass>::New();
  MyClass->SetM(M);
  
  vtkMatrix3x3* M2 = MyClass->GetM();
  
  vtkstd::cout << "M2: " << *M2 << vtkstd::endl;
  return 0;
}
