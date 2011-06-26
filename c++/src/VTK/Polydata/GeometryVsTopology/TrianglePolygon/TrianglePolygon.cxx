#include <vtkCellArray.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkTriangle.h>

int main()
{
  //setup points (geometry)
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  Points->InsertNextPoint ( 1.0, 0.0, 0.0 );
  Points->InsertNextPoint ( 0.0, 0.0, 0.0 );
  Points->InsertNextPoint ( 0.0, 1.0, 0.0 );

  vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();

  //create a triangle on the three points in the polydata
  vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();

  //Unfortunately in this simple example the following lines are ambiguous.
  //The first 0 is the index of the triangle vertex which is ALWAYS 0-2.
  //The second 0 is the index into the point (geometry) array, so this can range from 0-(NumPoints-1)
  //i.e. a more general statement is triangle->GetPointIds()->SetId(0, PointId);
  triangle->GetPointIds()->SetId ( 0, 0 );
  triangle->GetPointIds()->SetId ( 1, 1 );
  triangle->GetPointIds()->SetId ( 2, 2 );

  //add the triangle to the list of triangles (in this case there is only 1)
  triangles->InsertNextCell ( triangle );

  //create a polydata object
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  //add the geometry and topology to the polydata
  polydata->SetPoints ( Points );
  polydata->SetPolys ( triangles );

  //write the polydata to a file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName ( "Triangle.vtp" );
  writer->SetInput ( polydata );
  writer->Write();

}
