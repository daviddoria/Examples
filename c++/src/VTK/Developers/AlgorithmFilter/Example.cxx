#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>

#include "vtkAlgorithmFilter.h"


int main (int argc, char *argv[])
{
  vtkSmartPointer<vtkPolyData> polydata =
    vtkSmartPointer<vtkPolyData>::New();
    
  vtkSmartPointer<vtkAlgorithmFilter> filter = 
      vtkSmartPointer<vtkAlgorithmFilter>::New();
//  filter->SetInputConnection(inputTest->GetProducerPort());
  filter->Update();

  vtkUnstructuredGrid* ug = filter->GetOutput();
  
  return EXIT_SUCCESS;
}
