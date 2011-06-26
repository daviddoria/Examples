#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>

#include "vtkTestFilter.h"

int main (int argc, char *argv[])
{
  vtkSmartPointer<vtkPolyData> pd =
    vtkSmartPointer<vtkPolyData>::New();
  
  vtkSmartPointer<vtkTestFilter> filter = 
      vtkSmartPointer<vtkTestFilter>::New();
//  filter->SetInputConnection(inputTest->GetProducerPort());
  filter->Update();

  vtkUnstructuredGrid* ug = filter->GetOutput();
  
  return EXIT_SUCCESS;
}
