#include <vtkSmartPointer.h>

#include "vtkTestSource.h"
#include "vtkTest.h"

int main (int argc, char *argv[])
{
  vtkTestSource* source = vtkTestSource::New();
  source->Update();
  
  vtkTest* test = source->GetOutput();
  vtkstd::cout << test->GetValue() << vtkstd::endl;
  
  return 0;
}
