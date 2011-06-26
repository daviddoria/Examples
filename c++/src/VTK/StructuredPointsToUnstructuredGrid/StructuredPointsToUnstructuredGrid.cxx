#include <vtkSmartPointer.h>
#include <vtkStructuredPoints.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkUnstructuredGrid.h>

int main(int argc, char *argv[])
{
  //create a structuredpoints
  vtkSmartPointer<vtkStructuredPoints> structuredPoints = 
      vtkSmartPointer<vtkStructuredPoints>::New();
  
  //specify the size of the image data
  structuredPoints->SetDimensions(2,3,1);
  structuredPoints->SetNumberOfScalarComponents(1);
  structuredPoints->SetScalarTypeToDouble();
  
  int* dims = structuredPoints->GetDimensions();
    
  cout << "Dims: " << " x: " << dims[0] << " y: " << dims[1] << " z: " << dims[2] << endl;
  
  cout << "Number of points: " << structuredPoints->GetNumberOfPoints() << endl;
  cout << "Number of cells: " << structuredPoints->GetNumberOfCells() << endl;
  
  //fill every entry of the image data with "2.0"
  for (int z = 0; z < dims[2]; z++)
    {
    for (int y = 0; y < dims[1]; y++)
      {
      for (int x = 0; x < dims[0]; x++)
        {
        double* pixel = static_cast<double*>(structuredPoints->GetScalarPointer(x,y,z));
        pixel[0] = 2.0;
        }
      }
    }
  
  
  //copy the points from the structuredpoints to the unstructuredgrid
  vtkSmartPointer<vtkPoints> points = 
      vtkSmartPointer<vtkPoints>::New();
  
  //retrieve the entries from the grid and print them to the screen
  unsigned int counter = 0;
  for(vtkIdType i = 0; i < structuredPoints->GetNumberOfPoints(); i++)
    {
    double p[3];
    structuredPoints->GetPoint(i, p);
    
    points->InsertNextPoint(p);
    }
  
  vtkSmartPointer<vtkUnstructuredGrid> ug = 
    vtkSmartPointer<vtkUnstructuredGrid>::New();
  ug->SetPoints(points);
  
  vtkSmartPointer<vtkXMLUnstructuredGridWriter> writer = 
      vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
  writer->SetFileName("output.vtu");
  writer->SetInput(ug);
  writer->Write();
  
  return EXIT_SUCCESS;
}
