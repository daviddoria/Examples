#include <iostream>

#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

int main ( int argc, char *argv[] )
{
  
  vtkSmartPointer<vtkPoints> Points = vtkSmartPointer<vtkPoints>::New();
  
  for ( unsigned int i = 0; i < 10; ++i )
  {
    Points->InsertNextPoint ( drand48(), drand48(), drand48() );
  }

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  polydata->SetPoints(Points);

  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("test.vtp");
  writer->SetInput(polydata);
  writer->Write();

  std::cout << "Finished writing vtp file." << std::endl;

  return EXIT_SUCCESS;
}