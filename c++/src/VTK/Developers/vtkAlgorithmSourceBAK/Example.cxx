#include <vtkSmartPointer.h>

#include "vtkTestSource.h"
#include "vtkTest.h"

int main (int argc, char *argv[])
{
  if(argc != 2)
    {
    vtkstd::cout << "Required arguments: Filename" << vtkstd::endl;
    exit(-1);
    }
    
  vtkstd::string InputFilename = argv[1];
  
  vtkSmartPointer<vtkTestSource> source = vtkSmartPointer<vtkTestSource>::New();
  source->Update();
  
  vtkTest* test = source->GetOutput();
  vtkstd::cout << "Value: " << test->GetValue() << vtkstd::endl;
  
  return 0;
}
