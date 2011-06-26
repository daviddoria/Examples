#include <vtkPoints.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{
  
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer< vtkPoints >::New();
  vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
 
  for(unsigned int i = 0; i < 20; i++)
    {
    vtkIdType pid[1];
    pid[0] = points->InsertNextPoint(drand48(), drand48(), drand48());
    vertices->InsertNextCell ( 1,pid );
    }
 
  vtkSmartPointer< vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  polydata->SetVerts(vertices);
 
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("points.vtp");
  writer->SetInput(polydata);
  writer->Write();
 
  return 0;
}
