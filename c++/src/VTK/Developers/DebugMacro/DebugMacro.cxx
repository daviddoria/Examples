#include <vtkSmartPointer.h>

#include "vtkTestClass.h"

int main(int, char*[])
{

  vtkSmartPointer<vtkTestClass> testClass =
    vtkSmartPointer<vtkTestClass>::New();
  testClass->DebugOn();
  testClass->Update();

  return EXIT_SUCCESS;
}
