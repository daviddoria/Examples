#include <vtkSmartPointer.h>

#include "vtkTestReader.h"
#include "vtkTest.h"

int main (int argc, char *argv[])
{
  if(argc != 2)
    {
    vtkstd::cout << "Required arguments: Filename" << vtkstd::endl;
    exit(-1);
    }
    
  vtkstd::string InputFilename = argv[1];
  
  vtkSmartPointer<vtkTestReader> reader = vtkSmartPointer<vtkTestReader>::New();
  reader->Update();
  
  vtkTest* test = reader->GetOutput();
  vtkstd::cout << "Value: " << test->GetValue() << vtkstd::endl;
  
  return 0;
}
