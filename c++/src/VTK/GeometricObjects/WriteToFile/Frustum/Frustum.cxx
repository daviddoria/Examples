#include <vtkFrustumSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkCamera.h>
#include <vtkPlanes.h>

int main(int argc, char *argv[])
{

  vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
  double planesArray[24];
  
  camera->GetFrustumPlanes(1, planesArray);
  
  vtkSmartPointer<vtkPlanes> planes = vtkSmartPointer<vtkPlanes>::New();
  planes->SetFrustumPlanes(planesArray);
  
  vtkSmartPointer<vtkFrustumSource> frustumSource = vtkSmartPointer<vtkFrustumSource>::New();
  frustumSource->SetPlanes(planes);
  frustumSource->Update();
  
  vtkPolyData* frustum = frustumSource->GetOutput();

  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInput(frustum);
  writer->SetFileName("frustum.vtp");
  writer->Write();

  return 0;
}