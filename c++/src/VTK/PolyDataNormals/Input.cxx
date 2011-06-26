#include <vtkSmartPointer.h>
#include <vtkFloatArray.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkXMLPolyDataReader.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkXMLPolyDataReader> reader = 
      vtkSmartPointer<vtkXMLPolyDataReader>::New();
  reader->SetFileName(argv[1]);
  reader->Update();
      
  {
  vtkSmartPointer<vtkFloatArray> normals = 
      vtkFloatArray::SafeDownCast(reader->GetOutput()->GetPointData()->GetArray("Normals"));
  if(normals)
    {
    cout << "Got normals by name." << endl;
    }
  }
  
  {
  vtkSmartPointer<vtkFloatArray> normals = 
      vtkFloatArray::SafeDownCast(reader->GetOutput()->GetPointData()->GetNormals());
  if(normals)
    {
    cout << "Got normals by special array." << endl;
    }
  }
  
  return EXIT_SUCCESS;
}
