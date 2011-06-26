//can't do this, this is an abstract class!

#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkRectilinearGrid.h>
#include <vtkRectilinearGridToPolyDataFilter.h>
#include <vtkPolyData.h>
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
  
  vtkSmartPointer<vtkRectilinearGridToPolyDataFilter> rectilinearGridToPolyDataFilter = 
      vtkSmartPointer<vtkRectilinearGridToPolyDataFilter>::New();
  rectilinearGridToPolyDataFilter->SetInputConnection(grid->GetProducerPort());
  rectilinearGridToPolyDataFilter->Update();
  
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = 
      vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("output.vtp");
  writer->SetInputConnection(rectilinearGridToPolyDataFilter->GetOutputPort());
  writer->Write();
  
  std::cout << "There are " << rectilinearGridToPolyDataFilter->GetOutput()->GetNumberOfCells() << " cells." << std::endl;
  
  return EXIT_SUCCESS;
}