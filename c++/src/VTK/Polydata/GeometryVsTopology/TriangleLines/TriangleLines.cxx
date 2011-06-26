#include <vtkPoints.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkSmartPointer.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>

int main()
{
  //Setup point coordinates
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  Points->InsertNextPoint ( 1.0, 0.0, 0.0 );
  Points->InsertNextPoint ( 0.0, 0.0, 1.0 );
  Points->InsertNextPoint ( 0.0, 0.0, 0.0 );

  //create a line between each pair of points
  vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
  line0->GetPointIds()->SetId ( 0,0 );
  line0->GetPointIds()->SetId ( 1,1 );

  vtkSmartPointer<vtkLine> line1 = vtkSmartPointer<vtkLine>::New();
  line1->GetPointIds()->SetId ( 0,1 );
  line1->GetPointIds()->SetId ( 1,2 );

  vtkSmartPointer<vtkLine> line2 = vtkSmartPointer<vtkLine>::New();
  line2->GetPointIds()->SetId ( 0,2 );
  line2->GetPointIds()->SetId ( 1,0 );

  //create a cell array to store the line in
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
  lines->InsertNextCell ( line0 );
  lines->InsertNextCell ( line1 );
  lines->InsertNextCell ( line2 );

  //create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  //add the points and lines to the polydata
  polydata->SetPoints ( Points );
  polydata->SetLines ( lines );

  //write the polydata to a file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName ( "TriangleLines.vtp" );
  writer->SetInput ( polydata );
  writer->Write();

}
