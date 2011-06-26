#include <vtkSmartPointer.h>

#include "vtkTestFilter.h"
#include "vtkTest.h"

int main (int argc, char *argv[])
{
  vtkSmartPointer<vtkTest> InputTest = vtkSmartPointer<vtkTest>::New();
  InputTest->SetValue(5.6);
  
  vtkSmartPointer<vtkTestFilter> filter = vtkTestFilter::New();
  filter->SetInput(InputTest);
  filter->Update();
  
  vtkTest* OutputTest = filter->GetOutput();
  vtkstd::cout << OutputTest->GetValue() << vtkstd::endl;
  
  return 0;
}
