#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkSphereSource.h>
#include <vtkPointData.h>
#include <vtkReverseSense.h>
#include <vtkFloatArray.h>

int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkSphereSource> sphereSource = 
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();
  
  vtkSmartPointer<vtkFloatArray> pointNormals = 
      vtkFloatArray::SafeDownCast(sphereSource->GetOutput()->GetPointData()->GetNormals());

  cout << endl << "Normals: " << endl;
  //display the first few normals
  for(unsigned int i = 0; i < 5; i++)
    {
    double pN[3];
    pointNormals->GetTuple(i, pN);
    cout << "Point normal " << i << ": " << pN[0] << " " << pN[1] << " " << pN[2] << endl;
    }

  vtkSmartPointer<vtkReverseSense> reverseSense = 
      vtkSmartPointer<vtkReverseSense>::New();
  reverseSense->SetInputConnection(sphereSource->GetOutputPort());
  reverseSense->ReverseNormalsOn();
  reverseSense->Update();
 
  
  vtkSmartPointer<vtkFloatArray> reversedNormals = 
      vtkFloatArray::SafeDownCast(reverseSense->GetOutput()->GetPointData()->GetNormals());
   
  cout << endl << "Reversed: " << endl;
  //display the first few normals to verify that they are flipped
  for(unsigned int i = 0; i < 5; i++)
    {
    double pN[3];
    reversedNormals->GetTuple(i, pN);
    cout << "Reversed normal " << i << ": " << pN[0] << " " << pN[1] << " " << pN[2] << endl;
    }
  
  return EXIT_SUCCESS;
}
