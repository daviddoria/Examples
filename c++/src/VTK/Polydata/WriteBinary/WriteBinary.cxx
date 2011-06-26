#include <vtkPoints.h>
#include <vtkXMLPolyDataWriter.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

int main ( int argc, char *argv[] )
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  
  for ( unsigned int i = 0; i < 10; ++i )
    {
    points->InsertNextPoint ( drand48(), drand48(), drand48() );
    }

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  polydata->SetPoints(points);

  vtkSmartPointer<vtkXMLPolyDataWriter> writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
  writer->SetFileName("test.vtp");
  writer->SetInput(polydata);
  //writer->SetDataModeToBinary();
  //writer->SetDataModeToAscii();
  writer->Write();

  cout << "Finished writing vtp file." << endl;

  return 0;
}


