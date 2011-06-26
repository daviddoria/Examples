#include <vtkPolyData.h>
#include <vtkSimplePointsWriter.h>
#include <vtkSphereSource.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource =
    vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();

  vtkSmartPointer<vtkSimplePointsWriter> writer =
    vtkSmartPointer<vtkSimplePointsWriter>::New();
  writer->SetFileName("test.xyz");
  writer->SetInputConnection(sphereSource->GetOutputPort());
  writer->Write();

  return EXIT_SUCCESS;
}