#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

#include "vtkTestReader.h"

int main (int argc, char *argv[])
{
  vtkSmartPointer<vtkTestReader> reader = vtkSmartPointer<vtkTestReader>::New();
  reader->Update();
  
  vtkPolyData* polydata = reader->GetOutput();

  vtkstd::cout << "Number of points: " << polydata->GetNumberOfPoints() << vtkstd::endl;
  return 0;
}
