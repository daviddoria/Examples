#include <vtkOrderedTriangulator.h>
#include <vtkPlaneSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolygon.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkXMLUnstructuredGridWriter.h>

int main ( int argc, char *argv[] )
{
  vtkSmartPointer<vtkPlaneSource> planeSource =
    vtkSmartPointer<vtkPlaneSource>::New();
  planeSource->SetResolution(20, 20);
  planeSource->Update();

  double bounds[6];
  planeSource->GetOutput()->GetBounds(bounds);
  vtkSmartPointer<vtkOrderedTriangulator> orderedTriangulator =
    vtkSmartPointer<vtkOrderedTriangulator>::New();
  orderedTriangulator->InitTriangulation(bounds, planeSource->GetOutput()->GetNumberOfPoints());
  orderedTriangulator->PreSortedOff();
  
  for(unsigned int i = 0; i < planeSource->GetOutput()->GetNumberOfPoints(); ++i)
    {
    double p[3];
    planeSource->GetOutput()->GetPoint(i,p);
    orderedTriangulator->InsertPoint(i, p, p, 0);
    }
  orderedTriangulator->Triangulate();
  
  vtkSmartPointer<vtkUnstructuredGrid> grid = vtkSmartPointer<vtkUnstructuredGrid>::New();
  grid->Allocate (1, 1);
  orderedTriangulator->AddTetras(0, grid);
  grid->SetPoints(planeSource->GetOutput()->GetPoints());
  
  vtkSmartPointer<vtkXMLUnstructuredGridWriter> writer =
    vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
  writer->SetInputConnection(grid->GetProducerPort());
  writer->SetFileName("output.vtu");
  writer->Write();
  
  return EXIT_SUCCESS;
}
