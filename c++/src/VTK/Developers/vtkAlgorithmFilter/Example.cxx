#include <vtkSmartPointer.h>

#include "vtkTestFilter.h"
#include "vtkTest.h"

int main (int argc, char *argv[])
{
  vtkSmartPointer<vtkTest> inputTest =
    vtkSmartPointer<vtkTest>::New();
  inputTest->SetValue(5.6);
  
  vtkSmartPointer<vtkTestFilter> filter =
    vtkTestFilter::New();
  filter->SetInputConnection(inputTest->GetProducerPort());
  filter->Update();
  
  vtkTest* outputTest = filter->GetOutput();
  std::cout << outputTest->GetValue() << std::endl;
  
  return 0;
}
