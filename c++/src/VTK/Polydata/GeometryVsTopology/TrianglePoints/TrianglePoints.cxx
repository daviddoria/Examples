#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>

int main()
{
  //create a set of points
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  Points->InsertNextPoint ( 1.0, 0.0, 0.0 );
  Points->InsertNextPoint ( 0.0, 0.0, 0.0 );
  Points->InsertNextPoint ( 0.0, 1.0, 0.0 );

  //create a polydata
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  //add the points to the polydata
  polydata->SetPoints ( Points );

  //write the polydata to a file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName ( "TrianglePoints.vtp" );
  writer->SetInput ( polydata );
  writer->Write();

  return 0;
}
