#include <vtkSmartPointer.h>
#include "MyClass.h"

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMyClass> Test = vtkSmartPointer<vtkMyClass>::New();

  return 0;
}
