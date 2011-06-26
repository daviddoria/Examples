#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyData.h>
#include <vtkDataObjectToTable.h>
#include <vtkTable.h>

int main(int argc, char *argv[])
{
  
  //Create a sphere
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();

  vtkSmartPointer<vtkDataObjectToTable> dataObjectToTable = 
      vtkSmartPointer<vtkDataObjectToTable>::New();
  dataObjectToTable->SetInputConnection(sphereSource->GetOutputPort());
  //dataObjectToTable->SetFieldType(vtkDataObjectToTable::POINT_DATA);
  //dataObjectToTable->SetFieldType(vtkDataObjectToTable::CELL_DATA);
  dataObjectToTable->Update();
  
  vtkSmartPointer<vtkTable> table = dataObjectToTable->GetOutput();
  table->Dump();
  
  return EXIT_SUCCESS;
}