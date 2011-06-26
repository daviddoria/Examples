#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCellArray.h>
#include <vtkGeometryFilter.h>
#include <vtkPointSource.h>

int main(int argc, char **argv)
{ 

  vtkSmartPointer<vtkUnstructuredGrid> unstructuredGrid = 
      vtkSmartPointer<vtkUnstructuredGrid>::New();
  
  vtkSmartPointer<vtkGeometryFilter> geometryFilter = 
      vtkSmartPointer<vtkGeometryFilter>::New();
  geometryFilter->SetInput(unstructuredGrid);
  geometryFilter->Update(); 
 
  vtkPolyData* polydata = geometryFilter->GetOutput();
  
  return EXIT_SUCCESS;
}