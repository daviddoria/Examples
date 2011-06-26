#include <vtkSmartPointer.h>

#include "vtkTestFilter.h"


int main (int argc, char *argv[])
{
  
  vtkSmartPointer<vtkTestFilter> filter = 
      vtkSmartPointer<vtkTestFilter>::New();
//  filter->SetInputConnection(inputTest->GetProducerPort());
  filter->Update();

  return EXIT_SUCCESS;
}
