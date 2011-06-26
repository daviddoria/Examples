#include <vtkSmartPointer.h>
#include <vtkPolygon.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>

int main(int argc, char *argv[])
{
  //setup points
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  Points->InsertNextPoint(0.0, 0.0, 0.0);
  Points->InsertNextPoint(1.0, 0.0, 0.0);
  Points->InsertNextPoint(1.0, 1.0, 0.0);
  Points->InsertNextPoint(0.0, 1.0, 0.0);
  //the last point is implied

  vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPolygon> polygon = vtkSmartPointer<vtkPolygon>::New();
  polygon->GetPointIds()->SetNumberOfIds(4); //make a quad
  polygon->GetPointIds()->SetId(0, 0);
  polygon->GetPointIds()->SetId(1, 1);
  polygon->GetPointIds()->SetId(2, 2);
  polygon->GetPointIds()->SetId(3, 3);
  //this is an unfortunate example, as you cannot tell which 0,1,2 or 3 is which!
  //A clearer statement is SetId(0, IndexOfPoint0)
  polygons->InsertNextCell(polygon);

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(Points);
  polydata->SetPolys(polygons);

  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("Polygon.vtp");
  writer->SetInput(polydata);
  writer->Write();

	return 0;
}
