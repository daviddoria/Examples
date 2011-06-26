#include <vtkSmartPointer.h>

#include "vtkTestSource.h"
#include "vtkTestA.h"
#include "vtkTestB.h"

int main (int argc, char *argv[])
{
  vtkTestSource* source = vtkTestSource::New();
  source->Update();
  
  vtkTestA* testa = source->GetOutputA();
  vtkTestB* testb = source->GetOutputB();
  
  vtkstd::cout << testa->GetValue() << vtkstd::endl;
  
  vtkstd::cout << testb->GetValue() << vtkstd::endl;
  
  return 0;
}
