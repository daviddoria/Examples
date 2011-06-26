#include <vtkSmartPointer.h>
#include <vtkTriangle.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkTriangle> T = vtkSmartPointer<vtkTriangle>::New();
	
  //setup points
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  Points->InsertNextPoint(1.0, 0.0, 0.0);
  Points->InsertNextPoint(0.0, 0.0, 0.0);
  Points->InsertNextPoint(0.0, 1.0, 0.0);

  vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
  triangle->GetPointIds()->SetId(0, 0);
  triangle->GetPointIds()->SetId(1, 1);
  triangle->GetPointIds()->SetId(2, 2);
  //this is an unfortunate example, as you cannot tell which 0,1, or 2 is which!
  //A clearer statement is SetId(0, IndexOfPoint0)
  triangles->InsertNextCell(triangle);

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(Points);
  polydata->SetPolys(triangles);

  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("Triangle.vtp");
  writer->SetInput(polydata);
  writer->Write();

  return 0;
}
