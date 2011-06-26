#include <vtkSmartPointer.h>
#include <vtkMatrix3x3.h>
	
int main(int argc, char *argv[])
{
  vtkSmartPointer<vtkMatrix3x3> M = vtkSmartPointer<vtkMatrix3x3>::New();
  
  M->SetElement(2,1,2.0); //set element (0,0) to 1.0
  
  cout << *M << endl;
  
  M->Transpose();
  
  cout << *M << endl;
  
  return EXIT_SUCCESS;
}
