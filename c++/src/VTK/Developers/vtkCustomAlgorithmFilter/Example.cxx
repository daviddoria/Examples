#include <vtkSmartPointer.h>

#include "vtkTestFilter.h"
#include "vtkTest.h"

int main (int argc, char *argv[])
{
  vtkSmartPointer<vtkTest> inputTest = 
      vtkSmartPointer<vtkTest>::New();
  inputTest->SetValue(5.6);
  std::cout << "Input value: " << inputTest->GetValue() << std::endl;
  
  vtkSmartPointer<vtkTestFilter> filter = 
      vtkSmartPointer<vtkTestFilter>::New();
  filter->SetInputConnection(inputTest->GetProducerPort());
  filter->Update();
  
  vtkTest* outputTest = filter->GetOutput();
  std::cout << "Output value: " << outputTest->GetValue() << std::endl;
  std::cout << "Input value is still: " << inputTest->GetValue() << std::endl;
  
  return EXIT_SUCCESS;
}
