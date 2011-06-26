#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkHexahedron.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkSmartPointer.h>
 
int main(int, char *[])
{
  //Setup the coordinates of eight points 
  //(the two faces must be in counter clockwise order as viewd from the outside)
  double P0[3] = {0.0, 0.0, 0.0};
  double P1[3] = {1.0, 0.0, 0.0};
  double P2[3] = {1.0, 1.0, 0.0};
  double P3[3] = {0.0, 1.0, 0.0};
  double P4[3] = {0.0, 0.0, 1.0};
  double P5[3] = {1.0, 0.0, 1.0};
  double P6[3] = {1.0, 1.0, 1.0};
  double P7[3] = {0.0, 1.0, 1.0};

 
  //Create the points
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(P0);
  points->InsertNextPoint(P1);
  points->InsertNextPoint(P2);
  points->InsertNextPoint(P3);
  points->InsertNextPoint(P4);
  points->InsertNextPoint(P5);
  points->InsertNextPoint(P6);
  points->InsertNextPoint(P7);
 
  //Create a hexahedron from the points
  vtkSmartPointer<vtkHexahedron> hex = vtkSmartPointer<vtkHexahedron>::New();
  hex->GetPointIds()->SetId(0,0);
  hex->GetPointIds()->SetId(1,1);
  hex->GetPointIds()->SetId(2,2);
  hex->GetPointIds()->SetId(3,3);
  hex->GetPointIds()->SetId(4,4);
  hex->GetPointIds()->SetId(5,5);
  hex->GetPointIds()->SetId(6,6);
  hex->GetPointIds()->SetId(7,7);
 
  //Add the hexahedron to a cell array
  vtkSmartPointer<vtkCellArray> hexs = vtkSmartPointer<vtkCellArray>::New();
  hexs->InsertNextCell(hex);
 
  //Add the points and hexahedron to an unstructured grid
  vtkSmartPointer<vtkUnstructuredGrid> uGrid =
    vtkSmartPointer<vtkUnstructuredGrid>::New();
  uGrid->SetPoints(points);
  uGrid->InsertNextCell(hex->GetCellType(), hex->GetPointIds());
 
  //Write the file
  vtkSmartPointer<vtkXMLUnstructuredGridWriter> writer =
    vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
  writer->SetInput(uGrid);
  writer->SetFileName("Hexahedron.vtu");
  writer->Write();
 
  return EXIT_SUCCESS;
}
