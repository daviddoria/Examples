#include <vtkSmartPointer.h>
#include <vtkCellData.h>
#include <vtkMath.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkXMLPolyDataReader.h>
#include <vtkFieldData.h>
 
int main(int argc, char *argv[])
{
  //get filename from command line
  vtkstd::string filename = argv[1]; //first command line argument
 
  //Read the file
  vtkSmartPointer<vtkXMLPolyDataReader> reader = 
      vtkSmartPointer<vtkXMLPolyDataReader>::New();
  cout << "Reading " << filename << endl;
  reader->SetFileName(filename.c_str());
  reader->Update();
 
  //extract the polydata
  vtkSmartPointer<vtkPolyData> polydata = reader->GetOutput();
 
  //get the number of points in the polydata
  vtkIdType idNumPointsInFile = polydata->GetNumberOfPoints();
 
  vtkSmartPointer<vtkDoubleArray> location = 
      vtkSmartPointer<vtkDoubleArray>::New();
 
  //compute the center of mass (here we just use a random value)
  double locationValue[3] = {vtkMath::Random(0.0,1.0), vtkMath::Random(0.0,1.0), vtkMath::Random(0.0,1.0)};
 
  location->SetNumberOfComponents(3);
  location->SetName("Location");
  location->InsertNextTuple(locationValue);
  //the data is added to FIELD data (rather than POINT data as usual)
  polydata->GetFieldData()->AddArray(location);
 
  vtkSmartPointer<vtkIntArray> intValue =
      vtkSmartPointer<vtkIntArray>::New();
  intValue->SetNumberOfComponents(1);
  intValue->SetName("MyIntValue");
  intValue->InsertNextValue(5);
  polydata->GetFieldData()->AddArray(intValue);
 
  return EXIT_SUCCESS;
}