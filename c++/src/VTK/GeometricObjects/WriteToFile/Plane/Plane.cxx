#include <vtkXMLPolyDataWriter.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

#include <vtkPlane.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
  planeSource->SetCenter(1.0, 0.0, 0.0);
  planeSource->SetNormal(1.0, 0.0, 0.0);
  planeSource->SetResolution(10,10);
  
  //vtkPolyData* plane = planeSource->GetOutput();
  vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
    
  //distance to a point
  /*
  double point[3] = {1.0, 2.0, 3.0};
  //double d = plane->DistanceToPlane(point);
  double n[3];
  double o[3];
  plane->GetNormal(n);
  plane->GetOrigin(o);
  double d = vtkPlane::DistanceToPlane(point, n, o);
  */
  
  //write the file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInput(planeSource->GetOutput());
  writer->SetFileName("plane.vtp");
  writer->Write();

  return 0;
}