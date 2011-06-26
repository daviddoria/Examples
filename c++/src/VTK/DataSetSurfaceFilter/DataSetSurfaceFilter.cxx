#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCellArray.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkPointSource.h>

int main(int argc, char **argv)
{ 

  vtkSmartPointer<vtkUnstructuredGrid> unstructuredGrid = 
      vtkSmartPointer<vtkUnstructuredGrid>::New();
  
  vtkSmartPointer<vtkDataSetSurfaceFilter> surfaceFilter = 
      vtkSmartPointer<vtkDataSetSurfaceFilter>::New();
  surfaceFilter->SetInput(unstructuredGrid);
  surfaceFilter->Update(); 
 
  vtkPolyData* polydata = surfaceFilter->GetOutput();
  
  return EXIT_SUCCESS;
}