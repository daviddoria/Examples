#include <vtkSmartPointer.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkLine.h>
#include <vtkPolyData.h>
#include <vtkXMLPolyDataWriter.h>

int main(int argc, char *argv[])
{
  //create three points. Will will join (Origin and P0) with a red line and (Origin and P1) with a green line
  double Origin[3] = {0.0, 0.0, 0.0};
  double P0[3] = {1.0, 0.0, 0.0};
  double P1[3] = {0.0, 1.0, 0.0};
  
  //create a vtkPoints object and store the points in it
  vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();
  pts->InsertNextPoint(Origin);
  pts->InsertNextPoint(P0);
  pts->InsertNextPoint(P1);
  
  //setup two colors - one for each line
  unsigned char red[3] = {255, 0, 0};
  unsigned char green[3] = {0, 255, 0};
  
  //setup the colors array
  vtkSmartPointer<vtkUnsignedCharArray> Colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
  Colors->SetNumberOfComponents(3);
  Colors->SetName("Colors");
  
  //add the colors we created to the colors array
  Colors->InsertNextTupleValue(red);
  Colors->InsertNextTupleValue(green);
  
  //Create the first line (between Origin and P0)
  vtkSmartPointer<vtkLine> line0 = vtkSmartPointer<vtkLine>::New();
  line0->GetPointIds()->SetId(0,0); //the second 0 is the index of the Origin in the vtkPoints
  line0->GetPointIds()->SetId(1,1); //the second 1 is the index of P0 in the vtkPoints
      
  //Create the second line (between Origin and P1)
  vtkSmartPointer<vtkLine> line1 = vtkSmartPointer<vtkLine>::New();
  line1->GetPointIds()->SetId(0,0); //the second 0 is the index of the Origin in the vtkPoints
  line1->GetPointIds()->SetId(1,2); //2 is the index of P1 in the vtkPoints
  
  //create a cell array to store the lines in and add the lines to it
  vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
  lines->InsertNextCell(line0);
  lines->InsertNextCell(line1);
  
  //create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> pdata = vtkSmartPointer<vtkPolyData>::New();

  //add the points to the dataset
  pdata->SetPoints(pts);

  //add the lines to the dataset
  pdata->SetLines(lines);

  //color the lines - associate the first component (red) of the colors array with the first component of the cell array (line 0) and the second component (green) of the colors array with the second component of the cell array (line 1)
  pdata->GetCellData()->AddArray(Colors);

  //write the file
  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetInput(pdata);
  writer->SetFileName("ColoredLines.vtp");
  writer->Write();

  return 0;
}
