#include <vtkSmartPointer.h>
#include <vtkDataSet.h>

#include "vtkTestSource.h"
//#include "vtkTest.h"

int main (int argc, char *argv[])
{
  
  //vtkSmartPointer<vtkTestSource> source = vtkSmartPointer<vtkTestSource>::New();
  vtkTestSource* source = vtkTestSource::New();
  source->Update();
  
  //vtkTest* test = source->GetOutput();
  //vtkstd::cout << "Value: " << test->GetValue() << vtkstd::endl;
  
  vtkDataSet* test = source->GetOutput();
  return 0;
}
