#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkQuad.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkSmartPointer.h>

int main(int argc, char *argv[])
{
  //create four points
  //must be in counter clockwise order
  double P0[3] = {0.0, 0.0, 0.0};
  double P1[3] = {1.0, 0.0, 0.0};
  double P2[3] = {1.0, 1.0, 0.0};
  double P3[3] = {0.0, 1.0, 0.0};
      
  //add the points to a vtkPoints object
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(P0);
  points->InsertNextPoint(P1);
  points->InsertNextPoint(P2);
  points->InsertNextPoint(P3);
  
  //create a line between the two points
  vtkSmartPointer<vtkQuad> quad = vtkSmartPointer<vtkQuad>::New();
  quad->GetPointIds()->SetId(0,0);
  quad->GetPointIds()->SetId(1,1);
  quad->GetPointIds()->SetId(2,2);
  quad->GetPointIds()->SetId(3,3);
  
  //create a cell array to store the line in
  vtkSmartPointer<vtkCellArray> quads = vtkSmartPointer<vtkCellArray>::New();
  quads->InsertNextCell(quad);

  //create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  //add the points to the dataset
  polydata->SetPoints(points);

  //add the lines to the dataset
  polydata->SetPolys(quads);

  //write the file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInput(polydata);
  writer->SetFileName("Quad.vtp");
  writer->Write();

  return 0;
}

