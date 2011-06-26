#include <vtkSmartPointer.h>
#include <vtkMatrix4x4.h>
	
void Copy();
void MultiplyPoint();
void Invert();

int main()
{
  //Copy();
  Invert();
  //MultiplyPoint();
  return 0;
}

void Copy()
{
  vtkSmartPointer<vtkMatrix4x4> M = vtkSmartPointer<vtkMatrix4x4>::New();
  M->SetElement(0,0,4.0);

  cout << "M:" << endl;
  cout << *M << endl;
  
  vtkSmartPointer<vtkMatrix4x4> M2 = vtkSmartPointer<vtkMatrix4x4>::New();
  M2->DeepCopy(M);
  
  cout << "M2:" << endl;
  cout << *M2 << endl;
 
}

void Invert()
{
  vtkSmartPointer<vtkMatrix4x4> M = vtkSmartPointer<vtkMatrix4x4>::New();
  M->SetElement(2,0,4.0);

  cout << "M:" << endl << *M;
    
  M->Invert();
  
  cout << "M:" << endl << *M;
  
}


void MultiplyPoint()
{
  vtkSmartPointer<vtkMatrix4x4> M = vtkSmartPointer<vtkMatrix4x4>::New();
  M->SetElement(0,0,4.0);

  cout << "M:" << endl << *M << endl;
  
  double p[4] = {1,2,3,4};
  cout << "p:" << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << endl;
  
  double pout[4];
  
  M->MultiplyPoint(p, pout);
  cout << "p:" << pout[0] << " " << pout[1] << " " << pout[2] << " " << pout[3] << endl;
  
}