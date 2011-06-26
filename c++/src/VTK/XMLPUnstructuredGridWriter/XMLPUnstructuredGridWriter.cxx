#include <vtkSmartPointer.h>
#include <vtkXMLPUnstructuredGridWriter.h>
#include <vtkPointSource.h>
#include <vtkDelaunay3D.h>

int main(int, char *[])
{
  vtkSmartPointer<vtkPointSource> pointSource =
    vtkSmartPointer<vtkPointSource>::New();
  pointSource->Update();

  vtkSmartPointer<vtkDelaunay3D> delaunay =
    vtkSmartPointer<vtkDelaunay3D>::New();
  delaunay->SetInputConnection(pointSource->GetOutputPort());
  delaunay->Update();

  vtkSmartPointer<vtkXMLPUnstructuredGridWriter> writer =
    vtkSmartPointer<vtkXMLPUnstructuredGridWriter>::New();
  writer->SetInputConnection(delaunay->GetOutputPort());
  writer->SetFileName("Test.pvtu");
  writer->SetNumberOfPieces(4);
  writer->Update();

  return EXIT_SUCCESS;
}
