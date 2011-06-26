#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkLine.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{
  //Create two points
  double p0[3] = {1.0, 0.0, 0.0};
  double p1[3] = {0.0, 1.0, 0.0};
      
  //Add the two points to a vtkPoints object
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(p0);
  points->InsertNextPoint(p1);
  
  //Create a line between the two points
  vtkSmartPointer<vtkLine> line = vtkSmartPointer<vtkLine>::New();
  line->GetPointIds()->SetId(0,0);
  line->GetPointIds()->SetId(1,1);
  
  //create a cell array to store the line in
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
  lines->InsertNextCell(line);

  //create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  //add the points to the dataset
  polydata->SetPoints(points);

  //add the lines to the dataset
  polydata->SetLines(lines);

  //write the file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInput(pdata);
  writer->SetFileName("Line.vtp");
  writer->Write();

  return 0;
}

