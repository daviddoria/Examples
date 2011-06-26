#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkDoubleArray.h>
#include <vtkTriangle.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkXMLPolyDataReader.h>

int main(int argc, char *argv[])
{
 
  //we will read Test.vtp
  std::string InputFilename = "Test.vtp";
 //read the file
  vtkXMLPolyDataReader* reader = vtkXMLPolyDataReader::New();
  reader->SetFileName(InputFilename.c_str());
  reader->Update();
	
  vtkPolyData* polydata = reader->GetOutput();
	
  vtkDoubleArray* TriangleArea = vtkDoubleArray::SafeDownCast(polydata->GetCellData()->GetArray("TriangleArea"));
 
  vtkstd::cout << "Triangle area: " << TriangleArea->GetValue(0) << std::endl;
  
  return 0;
}