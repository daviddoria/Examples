#include <vtkCellData.h>
#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkTriangle.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkXMLPolyDataWriter.h>

int main(int argc, char *argv[])
{
  //we will write the resulting file to Test.vtp so it can be inspected in Paraview
  vtkstd::string OutputFilename = "Test.vtp";

  //setup 3 points
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(1.0, 0.0, 0.0);
  points->InsertNextPoint(0.0, 0.0, 0.0);
  points->InsertNextPoint(0.0, 1.0, 0.0);

  //create a triangle
  vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
  triangle->GetPointIds()->SetId(0,0);
  triangle->GetPointIds()->SetId(1,1);
  triangle->GetPointIds()->SetId(2,2);
  
  //add the triangle to a cell array
  vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
  triangles->InsertNextCell(triangle);

  //setup data for the triangle (attach a value of 1.45 - this can be anything you wish to store with it)
  vtkSmartPointer<vtkDoubleArray> triangleData = vtkSmartPointer<vtkDoubleArray>::New();
  triangleData->SetNumberOfComponents(1); //we will have only 1 value associated with the triangle
  triangleData->SetName("TriangleData"); //set the name of the value
  triangleData->InsertNextValue(1.45); //set the actual value

  //create a polydata that contains the points, the triangle on those points, and the data array (value) we created for the triangle
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);
  polydata->SetPolys(triangles);
  polydata->GetCellData()->AddArray(triangleData);

  //write the file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInput(polydata);
  writer->SetFileName(OutputFilename.c_str());
  writer->Write();

  return 0;
}