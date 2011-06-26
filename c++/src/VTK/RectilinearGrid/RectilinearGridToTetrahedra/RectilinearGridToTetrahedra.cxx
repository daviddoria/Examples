#include <vtkSmartPointer.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkRectilinearGrid.h>
#include <vtkRectilinearGridToTetrahedra.h>
#include <vtkUnstructuredGrid.h>
#include <vtkMath.h>
#include <vtkDoubleArray.h>

int main(int, char *[])
{
  //create a grid
  vtkSmartPointer<vtkRectilinearGrid> grid = 
      vtkSmartPointer<vtkRectilinearGrid>::New();
  
  grid->SetDimensions(2,3,1);
  
  vtkSmartPointer<vtkDoubleArray> xArray = 
      vtkSmartPointer<vtkDoubleArray>::New();
  xArray->InsertNextValue(0.0);
  xArray->InsertNextValue(2.0);
  
  vtkSmartPointer<vtkDoubleArray> yArray = 
      vtkSmartPointer<vtkDoubleArray>::New();
  yArray->InsertNextValue(0.0);
  yArray->InsertNextValue(1.0);
  yArray->InsertNextValue(2.0);
  
  vtkSmartPointer<vtkDoubleArray> zArray = 
      vtkSmartPointer<vtkDoubleArray>::New();
  zArray->InsertNextValue(0.0);
  
  grid->SetXCoordinates(xArray);
  grid->SetYCoordinates(yArray);
  grid->SetZCoordinates(zArray);
  
  vtkSmartPointer<vtkRectilinearGridToTetrahedra> rectilinearGridToTetrahedra = 
      vtkSmartPointer<vtkRectilinearGridToTetrahedra>::New();
  rectilinearGridToTetrahedra->SetInputConnection(grid->GetProducerPort());
  rectilinearGridToTetrahedra->Update();
  
  vtkSmartPointer<vtkXMLUnstructuredGridWriter> writer = 
      vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
  writer->SetFileName("output.vtu");
  writer->SetInputConnection(rectilinearGridToTetrahedra->GetOutputPort());
  writer->Write();
  
  std::cout << "There are " << rectilinearGridToTetrahedra->GetOutput()->GetNumberOfCells() << " cells." << std::endl;
  
  return EXIT_SUCCESS;
}